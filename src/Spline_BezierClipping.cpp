#include "Spline.h"
#include <geometry/Triangle.h>
#include <vector>

namespace cg::spline
{

bool doBezierClipping(Intersection& hit,
    const Ray3f& ray,
    const vec4f buffer[],
    const uint32_t patch[16])
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

    // TODO project patch and call doBezierClipping2D

    return false;
}

// Just for reference:
// static inline
// float distanceFromLine(const vec2f& p, float a, float b, float c)
// {
//     return p.dot({a,b}) + c;
// }

/**
 * @brief Performs Bézier clipping on an already projected non-rational patch.
 * 
 * @param [out] hits Output vector to store all the intersection coordinates.
 * @param patch Bézier patch in 2D with control points (x, y).
 * @param tol Required precision for each dimension.
 * @retval Returns if an intersection was found.
 */
bool doBezierClipping2D(std::vector<vec2f>& hits,
    const vec2f patch[16],
    const vec2f tol = {1e-4, 1e-4})
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
            V0 = p.point(3,0) - p.point(0,0);
            V1 = p.point(3,3) - p.point(0,3);
        }
        else
        {
            V0 = p.point(0,3) - p.point(0,0);
            V1 = p.point(3,3) - p.point(3,0);
        }
        // consider a line ax + by + c = 0, L represents a line that intersects
        // the plane origin, therefore c = 0.
        // L = (a, b)
        L = (V0 + V1).versor(); // middle line

        // create distance patch
        for (int i = 0; i < D.size(); i++)
            // distance between the control point P_i and line L
            D[i] = p[i].dot(L);

        constexpr float over3[] { 0, 1/3, 2/3, 1 };

        // compute the region to be clipped
        float lower = 1, upper = 0;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                vec2f A { over3[j], D.point(i, j) };
                vec2f B { over3[j+1], D.point(i, j+1) };
                // find where distance is zero
                // L: A + Bt
                // Ay + By*t = 0 => t = Ay / (Ay - By), Ay != By
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
                    const float s = (1 - t)*A.x + t*B.x;
                    lower = std::min(lower, s);
                    upper = std::max(upper, s);
                }
            }

        }

        const float d = upper - lower;
        if (d < 0)
        { // clipping failed; no intersection
            st.pop();
        }
        else if (d > subdivision_threshold)
        { // clipped region too small; subdivide
            State s1;
            std::copy(e.patch, e.patch+16, s1.patch);
            if (e.cutside)
            {
                const auto hy = 0.5f * (e.max.y + e.min.y);
                s1.min = {e.min.x, hy};
                s1.max = e.max;
                e.max = {e.max.x, hy};
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
            // continue clipping
            if (e.cutside)
            {
                // recompute active region coordinates
                e.min.y = e.min.y + (lower / e.size.y);
                e.max.y = e.min.y + (upper / e.size.y);
                e.size.y = e.max.y - e.min.y;
                // clip
                subpatchV(e.patch, lower, upper);
            }
            else
            {
                e.min.x = e.min.x + (lower / e.size.x);
                e.max.x = e.min.x + (upper / e.size.x);
                e.size.x = e.max.x - e.min.x;
                subpatchU(e.patch, lower, upper);
            }
            e.cutside = !e.cutside;
        }

        if (e.size.x <= tol.x && e.size.y <= tol.y)
        { // intersection found
            const auto x = 0.5f * (e.max.x - e.min.x);
            const auto y = 0.5f * (e.max.y - e.min.y);
            hits.emplace_back(x, y, 0.0f);
            st.pop();
        }
    }

    return hits.empty() == false;
}

} // namespace cg::spline
