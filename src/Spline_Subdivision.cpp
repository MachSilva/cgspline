#include "Spline.h"
#include <geometry/Triangle.h>

namespace cg::spline
{

bool doSubdivision(Intersection& hit,
    const Ray3f& ray,
    const vec4f buffer[],
    const uint32_t patch[16],
    float threshold)
{
    struct State
    {
        vec2f min;
        vec2f max;
        bool cutside;
    };
    
    // static thread_local
    std::stack<State> st;
    float tmin, tmax;
    vec4f subp[16];

    // First iteration (no need to use `slice`)
    auto pb = boundingbox(buffer, patch);
    if (pb.intersect(ray, tmin, tmax) == false)
        return false;

    // Areas of interest
    st.push({ .min = vec2f( 0, 0), .max = vec2f(.5, 1), .cutside = 0 });
    st.push({ .min = vec2f(.5, 0), .max = vec2f( 1, 1), .cutside = 0 });

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
                        .min = vec2f(e.min.x, e.min.y),
                        .max = vec2f(      x, e.max.y),
                        .cutside = !e.cutside,
                    });
                    st.push({
                        .min = vec2f(      x, e.min.y),
                        .max = vec2f(e.max.x, e.max.y),
                        .cutside = !e.cutside,
                    });
                }
                else
                {
                    st.push({
                        .min = vec2f(e.min.x, e.min.y),
                        .max = vec2f(e.max.x,       y),
                        .cutside = !e.cutside,
                    });
                    st.push({
                        .min = vec2f(e.min.x,       y),
                        .max = vec2f(e.max.x, e.max.y),
                        .cutside = !e.cutside,
                    });
                }
            }
        }
    }

    return found;
}


} // namespace cg::spline
