#include "Spline.h"
#include <geometry/Triangle.h>

namespace cg::spline
{

mat3f rotation(const vec3f& v, float angle)
{
    using std::cos, std::sin;
    const auto c = cos(angle);
    const auto s = sin(angle);
    const auto c1 = 1.0f - c;
    const auto xx = v.x * v.x;
    const auto yy = v.y * v.y;
    const auto zz = v.z * v.z;
    const auto xy = v.x * v.y;
    const auto xz = v.x * v.z;
    const auto yz = v.y * v.z;
    const auto sx = s * v.x;
    const auto sy = s * v.y;
    const auto sz = s * v.z;
    return mat3f
    { // columns
        {c1 * xx + c,  c1 * xy + sz, c1 * xz - sy},
        {c1 * xy - sz, c1 * yy + c,  c1 * yz + sx},
        {c1 * xz + sy, c1 * yz - sx, c1 * zz + c}
    };
}

bool doBezierClipping(Intersection& hit,
    const Ray3f& ray,
    const vec4f buffer[],
    const uint32_t patch[16],
    float tol)
{
    using std::acos, std::cos, std::sin;
    // static thread_local
    std::vector<vec2f> hits;

    // Project patch and call doBezierClipping2D
    const vec3f d = ray.direction;
    const vec3f axis = d.cross({0,0,-1}).versor();
    // const float a = 0.5f * acos(d.dot({0,0,-1}));
    // const quatf q (axis * sin(a), cos(a));
    // projection matrix
    // const mat4f M = mat4f(q) * mat4f(mat3f(1.0f), -ray.origin);
    const mat3f R = rotation(axis, acos(d.dot({0,0,-1})));
    const mat4f M = mat4f(R) * mat4f(mat3f(1.0f), -ray.origin);

    auto projectedP0 = M.transform3x4(ray.origin);
    auto projectedP1 = M.transform3x4(ray.origin + d);

    vec2f projectedpatch[16];
    for (int i = 0; i < 16; i++)
    {
        vec4f p = M * buffer[patch[i]];
        projectedpatch[i] = {p.x / p.w, p.y / p.w};
    }

    bool found = false;
    if (doBezierClipping2D(hits, projectedpatch))
    {
        PatchRef S (buffer, patch);
        for (auto& e : hits)
        {
            vec4f p = interpolate(S, e.x, e.y);
            float t = (project(p) - ray.origin).length();
            if (t < hit.distance && t > 0)
            {
                hit.p = {e.x, e.y, 0};
                hit.distance = t;
                found = true;
            }
        }
    }
    return found;
}

// Just for reference:
// static inline
// float distanceFromLine(const vec2f& p, float a, float b, float c)
// {
//     return p.dot({a,b}) + c;
// }

bool doBezierClipping2D(std::vector<vec2f>& hits,
    const vec2f patch[16],
    float tol)
{
    struct State
    {
        vec2f patch[16];
        vec2f min;
        vec2f max;
        vec2f size; // state.max - state.min
        bool cutside; // 0: u dimension; 1: v dimension
    };

    // static thread_local
    std::stack<State> st;

    st.emplace(State{.min = {0,0}, .max = {1,1}, .size = {1,1},
        .cutside = 0});
    std::copy(patch, patch+16, st.top().patch);
    hits.clear();

    float distancePatch[16];
    auto D = Patch(distancePatch);

    const auto eps = std::numeric_limits<float>::epsilon();
    const auto subdivision_threshold = 0.8f;
    while (!st.empty())
    {
        State &e = st.top();
        auto p = Patch(e.patch);
        vec2f V0, V1, L;
        if (e.cutside)
        {
            V0 = p.point(0,3) - p.point(0,0);
            V1 = p.point(3,3) - p.point(3,0);
        }
        else
        {
            V0 = p.point(3,0) - p.point(0,0);
            V1 = p.point(3,3) - p.point(0,3);
        }
        // consider a line ax + by + c = 0, L represents a line that intersects
        // the plane origin, therefore c = 0.
        // L = (a, b)
        L = (V0 + V1).versor(); // middle line

        // create distance patch
        for (int i = 0; i < D.size(); i++)
            // distance between the control point P_i and line L
            D[i] = p[i].dot(L);

        constexpr float over3[] { 0.0, 1.0/3.0, 2.0/3.0, 1.0 };

        // compute the region to be clipped
        float lower = 1, upper = 0;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                const float da = e.cutside
                    ? D.point(i, j)
                    : D.point(j, i);
                const float db = e.cutside
                    ? D.point(i, j+1)
                    : D.point(j+1, i);
                vec2f A { over3[j], da };
                vec2f B { over3[j+1], db };
                // find where distance is zero
                // L: A + Vt,   V = (B - A)
                // Ay + Vy*t = 0 => t = Ay / (Ay - By), Ay != By
                if (std::abs(A.y - B.y) <= eps)
                { // what is the best we could do here?
                    if (std::abs(A.y) <= eps)
                    {
                        lower = std::min(lower, A.x);
                        upper = std::max(upper, B.x);
                    }
                }
                else
                {
                    const float t = A.y / (A.y - B.y);
                    if (t >= 0.0f && t <= 1.0f)
                    {
                        const float s = (1 - t)*A.x + t*B.x;
                        lower = std::min(lower, s);
                        upper = std::max(upper, s);
                    }
                }
            }
        }

        const float d = upper - lower;
        if (d < 0)
        { // clipping failed; no intersection
            st.pop();
            // this iteration is already over
            // we must not allow the code to access an invalid memory
            continue;
        }
        else if (d > subdivision_threshold)
        { // clipped region too small; subdivide
            State s1;
            std::copy(e.patch, e.patch+16, s1.patch);
            if (e.cutside)
            {
                // e: first half; s1: second half of the patch
                const auto hy = 0.5f * (e.max.y + e.min.y);
                s1.min = {e.min.x, hy};
                s1.max = e.max;
                e.max = {e.max.x, hy};
                // e.min = e.min;
                subpatchV(e.patch, 0.0f, 0.5f);
                subpatchV(s1.patch, 0.5f, 1.0f);
            }
            else
            {
                const auto hx = 0.5f * (e.max.x + e.min.x);
                s1.min = {hx, e.min.y};
                s1.max = e.max;
                e.max = {hx, e.max.y};
                subpatchU(e.patch, 0.0f, 0.5f);
                subpatchU(s1.patch, 0.5f, 1.0f);
            }
            s1.cutside = e.cutside = !e.cutside;
            s1.size = s1.max - s1.min;
            e.size = e.max - e.min;
            // st.push(std::move(e)); // already in stack
            st.push(std::move(s1));
        }
        else // just continue clipping
        {
            // lower = 0.99f * lower;
            // upper = 0.99f * upper + 0.01f;
            // continue clipping
            if (e.cutside)
            {
                // recompute active region coordinates
                e.min.y = e.min.y + (lower * e.size.y);
                e.max.y = e.max.y - ((1 - upper) * e.size.y);
                e.size.y = e.max.y - e.min.y;
                // clip
                subpatchV(e.patch, lower, upper);
            }
            else
            {
                e.min.x = e.min.x + (lower * e.size.x);
                e.max.x = e.max.x - ((1 - upper) * e.size.x);
                e.size.x = e.max.x - e.min.x;
                subpatchU(e.patch, lower, upper);
            }
            e.cutside = !e.cutside;
        }

        float sx = (p.point(3,0) - p.point(0,0)).length();
        float sy = (p.point(0,3) - p.point(0,0)).length();
        if (sx <= tol && sy <= tol)
        { // intersection found
            const auto x = 0.5f * (e.max.x + e.min.x);
            const auto y = 0.5f * (e.max.y + e.min.y);
            hits.emplace_back(vec2f{x, y});
            st.pop();
        }
    }

    return hits.empty() == false;
}

} // namespace cg::spline
