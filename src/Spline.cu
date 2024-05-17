#include "Spline.h"
#include <geometry/Triangle.h>

#include <cuda/std/cmath>
#include <cuda/std/cstdint>

namespace cg::spline
{

HOST DEVICE
vec3 normal(const PatchRef<vec4,uint32_t> &s, float u, float v)
{
    constexpr float e = SPL_NORMAL_FIXVALUE;

    vec4 derU = derivativeU(s, u, v);
    vec4 derV = derivativeV(s, u, v);
    // Fix for degenerated patches.
    if (derU.x == 0 && derU.y == 0 && derU.z == 0)
    {
        derU = derivativeU(s, u, std::abs(v - e));
    }
    if (derV.x == 0 && derV.y == 0 && derV.z == 0)
    {
        derV = derivativeV(s, std::abs(u - e), v);
    }
    return vec3(derU).cross(vec3f(derV));
}

} // namespace cg::spline
