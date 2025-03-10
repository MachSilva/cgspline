#include "Spline.h"
#include <geometry/Triangle.h>

#include <stack>
#include <cuda/std/array>
#include <cuda/std/cstddef>
#include <cuda/std/limits>
#include <cuda/std/utility>
#include "rt/Frame.h"
#include "rt/StaticConfig.h"
#include "SplineMat.h"

namespace cg::spline
{

static DEVICE
bool doSubdivision2D_device(std::predicate<vec2> auto onHit,
    const vec2 patch[16],
    float tol)
{
    namespace std = ::cuda::std;
    using namespace mat;

    bool found = false;
    uint32_t trail = 0;
    uint16_t trailU = 0;
    uint16_t trailV = 0;
    int depth = -1;
    bool reload = false;

    vec2 buffer[16];
    Patch p (buffer);

    for (int i = 0; i < 16; i++)
        buffer[i] = patch[i];

    do
    {
        const float s0 = scalbnf(1, depth/2);
        const float s1 = scalbnf(1, (depth-1)/2);
        const vec2 emin {trailU * s0, trailV * s1};
        const vec2 emax {emin.x + s0, emin.y + s1};

        // new here? (postponed & 1 == 0)
        // already evaluated left node and going to right node (postponed & 1 == 1)

        if (reload)
            patchCopy(buffer, patch, emin, emax);
        // bounding box
        vec2 bmin = buffer[0]; // bounds min
        vec2 bmax = buffer[0]; // bounds max
        #pragma unroll
        for (int i = 1; i < 16; i++)
        {
            vec2& e = buffer[i];
            bmin = min(bmin, e);
            bmax = max(bmax, e);
        }
        // contains the origin?
        if (bmin.x < 0 && bmin.y < 0 && bmax.x > 0 && bmax.y > 0)
        {
            // check tolerance
            float d0 = (p.point(3,0) - p.point(0,3)).length();
            float d1 = (p.point(3,3) - p.point(0,0)).length();
            if ((d0 < tol && d1 < tol) || depth < -31)
            {
                found |= approximateIntersection(onHit, p, emin, emax, emax - emin);
            }
            else // subdivide
            {
                reload = false;
                if (depth & 1) // next to advance will be s0 (u-dimension)
                {
                    subpatchU(buffer, 0.0f, 0.5f);
                    trailU <<= 1;
                }
                else
                {
                    subpatchV(buffer, 0.0f, 0.5f);
                    trailV <<= 1;
                }
                trail <<= 1;
                --depth;
                continue;
            }
        }

        // next or go back
        reload = true;
        int ones = __clz(__brev(~trail));
        if (depth & 1) // s1 (v-dimension) was cut in half
        {
            trailU >>= (ones >> 1);
            trailV >>= (ones >> 1) + (ones & 1);
            ones & 1 ? (trailU |= 1) : (trailV |= 1); // +1 also works
        }
        else // s0 (u-dimension) was cut in half
        {
            trailV >>= (ones >> 1);
            trailU >>= (ones >> 1) + (ones & 1);
            ones & 1 ? (trailV |= 1) : (trailU |= 1);
        }
        trail = (trail >> ones) | 1; // +1 also works
        depth += ones;
    }
    while (depth < 0);

    return found;
}

DEVICE
bool doSubdivision_device(Intersection& hit,
    const Ray3f& ray,
    const vec4 buffer[],
    const uint32_t patch[16],
    float tol)
{
    // project patch into a 2D plane perpendicular to the ray direction
    vec2 patch2D[16];

    // find two planes whose intersection is a line that contains the ray
    const auto& d = ray.direction;

    const vec3 axis0 = d.cross(d + vec3(0,0,8)).versor();
    const vec3 axis1 = d.cross(axis0).versor();

    // quat q (vec3(0,0,1).cross(d), 1 + d.z);
    // q.normalize();
    // const vec3 axis0 = q.rotate(vec3(1,0,0)).versor();
    // const vec3 axis1 = q.rotate(vec3(0,1,0)).versor();

    for (int i = 0; i < 16; i++)
    {
// #if !defined(__CUDA_ARCH__)
        const vec4 P = buffer[patch[i]];
// #else
//         const vec4 P
//         {
//             __ldg((const float4*) buffer + __ldg(patch+i))
//         };
// #endif
        const vec3 Q {P.x, P.y, P.z};
        patch2D[i] =
        {
            distance(axis0, ray.origin, Q),
            distance(axis1, ray.origin, Q)
        };
    }

    PatchRef S (buffer, patch);
    auto onHit = [&](vec2 e) -> bool
    {
        auto V = project(mat::interpolate(S, e.x, e.y)) - ray.origin;
        float t = V.length();

        if (t < hit.distance && V.dot(ray.direction) > 0)
        {
            hit.distance = t;
            hit.p = {e.x, e.y};
            return true;
        }
        return false;
    };
    return doSubdivision2D_device(onHit, patch2D, tol);
}


bool doSubdivision_host(Intersection& hit,
    const Ray3f& ray,
    const vec4 buffer[],
    const uint32_t patch[16],
    float threshold)
{
    struct State
    {
        vec2 min;
        vec2 max;
        bool cutside;
    };
    
    std::stack<State> st;
    float tmin, tmax;
    vec4 subp[16];

    // First iteration (no need to use `slice`)
    auto pb = boundingbox(buffer, patch);
    if (pb.intersect(ray, tmin, tmax) == false)
        return false;

    // Areas of interest
    st.push({ .min = vec2( 0, 0), .max = vec2(.5, 1), .cutside = 0 });
    st.push({ .min = vec2(.5, 0), .max = vec2( 1, 1), .cutside = 0 });

    bool found = false;
    while (!st.empty())
    {
        State e = st.top();
        st.pop();

        // intersection with bounds
        // if intersected
        // - if bounds size < threshold
        //   - report hit and continue
        // - else
        //   - subdivide and push into stack
        // else
        // - discard

        pb = subpatchBoundingbox(subp, buffer, patch, e.min.x, e.max.x, e.min.y, e.max.y);
        if (pb.intersect(ray, tmin, tmax))
        {
            auto x = .5f * (e.min.x + e.max.x);
            auto y = .5f * (e.min.y + e.max.y);
            if ((e.max.x - e.min.x) < threshold && (e.max.y - e.min.y) < threshold)
            {
                // report hit
                // intersect triangles
                auto p00 = project(subp[0]);   // (u,v) = (0,0)
                auto p10 = project(subp[3]);   // (1,0)
                auto p01 = project(subp[12]);  // (0,1)
                auto p11 = project(subp[15]);  // (1,1)
                vec3f uvw, uvw1;
                float t, t1;
                bool b0, b1;
                b0 = triangle::intersect(ray, p10, p01, p00, uvw, t);
                b1 = triangle::intersect(ray, p01, p10, p11, uvw1, t1);
                // select nearest triangle
                if (b0)
                {
                    if (b1 && t1 < t)
                    {
                        t = t1;
                        uvw.x = 1 - uvw1.x;
                        uvw.y = 1 - uvw1.y;
                    }
                    // else only one hit at t0;
                }
                else if (b1)
                {
                    t = t1;
                    uvw.x = 1 - uvw1.x;
                    uvw.y = 1 - uvw1.y;
                    b0 = true; // at least one intersection exists in uvw
                }

                if (b0 && t < hit.distance)
                {
                    hit.distance = t;
                    hit.p.x = x;
                    hit.p.y = y;
                    found = true;
                }
            }
            else
            {
                // subdivide
                if (e.cutside)
                {
                    st.push({
                        .min = vec2(e.min.x, e.min.y),
                        .max = vec2(      x, e.max.y),
                        .cutside = !e.cutside,
                    });
                    st.push({
                        .min = vec2(      x, e.min.y),
                        .max = vec2(e.max.x, e.max.y),
                        .cutside = !e.cutside,
                    });
                }
                else
                {
                    st.push({
                        .min = vec2(e.min.x, e.min.y),
                        .max = vec2(e.max.x,       y),
                        .cutside = !e.cutside,
                    });
                    st.push({
                        .min = vec2(e.min.x,       y),
                        .max = vec2(e.max.x, e.max.y),
                        .cutside = !e.cutside,
                    });
                }
            }
        }
    }

    return found;
}

HOST DEVICE
bool doSubdivision(Intersection& hit,
    const Ray3f& ray,
    const vec4 buffer[],
    const uint32_t patch[16],
    float threshold)
{
#ifndef __CUDA_ARCH__
    return doSubdivision_host(hit, ray, buffer, patch, threshold);
#else
    return doSubdivision_device(hit, ray, buffer, patch, threshold);
#endif
}

} // namespace cg::spline

