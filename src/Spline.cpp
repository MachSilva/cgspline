#include "Spline.h"

namespace cg::spline
{

// Instantiate templates
template
Bounds3f subpatchBoundingbox<vec3f,float,uint32_t>(
    const vec3f*, const uint32_t*, float, float, float, float);

template
Bounds3f subpatchBoundingbox<vec4f,float,uint32_t>(
    const vec4f*, const uint32_t*, float, float, float, float);

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

    // First iteration (no need to use `slice`)
    auto pb = boundingbox(std::views::transform(
        std::views::counted(patch, 16),
        [&](uint32_t i) -> const vec4f& { return buffer[i]; }
    ));
    if (! pb.intersect(ray, tmin, tmax))
        return false;

    // Areas of interest
    st.push({ .min = vec2f( 0, 0), .max = vec2f(.5, 1), .cutside = 0});
    st.push({ .min = vec2f(.5, 0), .max = vec2f( 1, 1), .cutside = 0});

    bool found = false;
    while (!st.empty())
    {
        State e = st.top();
        st.pop();

        // fprintf(stderr,
        //     "doSubdivision: search area u = [%.05f,%.05f], v = [%.05f,%.05f]\n",
        //     e.min.x, e.max.x, e.min.y, e.max.y
        // );

        // intersection with bounds
        // if intersected
        // - if bounds size < threshold
        //   - report hit and continue
        // - else
        //   - subdivide and push into stack
        // else
        // - discard

        pb = subpatchBoundingbox(buffer, patch, e.min.x, e.max.x, e.min.y, e.max.y);
        if (pb.intersect(ray, tmin, tmax))
        {
            auto x = .5f * (e.min.x + e.max.x);
            auto y = .5f * (e.min.y + e.max.y);
            // auto s = pb.size();
            if ((e.max.x - e.min.x) < threshold && (e.max.y - e.min.y) < threshold)
            {
                // report hit
                auto t = .5f * (tmin + tmax);

#ifndef NDEBUG
                fprintf(stderr,
                    "doSubdivision: hit at (u = %.05f, v = %.05f), distance: %.02f%s best: %.02f"
                    ", patch %04X\n",
                    x, y, t, (t < hit.distance ? " <" : ", "), hit.distance,
                    (reinterpret_cast<std::ptrdiff_t>(patch)) & 0xFFFF
                );
#endif
                if (t < hit.distance)
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

