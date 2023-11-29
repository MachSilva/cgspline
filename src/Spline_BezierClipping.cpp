#include "Spline.h"
#include <geometry/Triangle.h>

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


    return false;
}

} // namespace cg::spline
