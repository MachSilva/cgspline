#include "Spline.h"
#include <geometry/Triangle.h>

#include <cuda/std/cmath>
#include <cuda/std/cstdint>

namespace cg::spline
{

HOST DEVICE
vec3f normal(const PatchRef<vec4f,uint32_t> &s, float u, float v)
{
    constexpr float e = SPL_NORMAL_FIXVALUE;

    vec4f derU = derivativeU(s, u, v);
    vec4f derV = derivativeV(s, u, v);
    // Fix for degenerated patches.
    if (derU.x == 0 && derU.y == 0 && derU.z == 0)
    {
        derU = derivativeU(s, u, std::abs(v - e));
    }
    if (derV.x == 0 && derV.y == 0 && derV.z == 0)
    {
        derV = derivativeV(s, std::abs(u - e), v);
    }
    return vec3f::cross(vec3f(derU), vec3f(derV));
}

} // namespace cg::spline
