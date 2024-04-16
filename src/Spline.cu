#include "Spline.h"
#include <geometry/Triangle.h>

namespace cg::spline
{

// Instantiate templates
// template
// Bounds3f subpatchBoundingbox<vec3f,float,uint32_t>(
//     const vec3f*, const uint32_t*, float, float, float, float);

// template
// Bounds3f subpatchBoundingbox<vec4f,float,uint32_t>(
//     const vec4f*, const uint32_t*, float, float, float, float);

vec4f& deCasteljau(vec4f p[], float u)
{
    p[0] += u * (p[1] - p[0]);
    p[1] += u * (p[2] - p[1]);
    p[2] += u * (p[3] - p[2]);

    p[0] += u * (p[1] - p[0]);
    p[1] += u * (p[2] - p[1]);

    p[0] += u * (p[1] - p[0]);
    return p[0];
}

vec4f& deCasteljauDx(vec4f p[], float u)
{
    p[0] = p[1] - p[0];
    p[1] = p[2] - p[1];
    p[2] = p[3] - p[2];

    p[0] += u * (p[1] - p[0]);
    p[1] += u * (p[2] - p[1]);
    
    p[0] += u * (p[1] - p[0]);
    return p[0];
}

// vec4f derivativeU(const PatchRef<vec4f,uint32_t> &s, float u, float v)
// {
//     vec4f p[MAX_POINTS];
//     vec4f q[MAX_POINTS];

//     auto sizeU = s.shape(0);
//     auto sizeV = s.shape(1);
//     assert(sizeU <= MAX_POINTS && sizeV <= MAX_POINTS);

//     for (auto j = 0; j < sizeV; j++)
//     {
//         for (auto i = 0; i < sizeU; i++)
//             p[i] = s.point(i, j);
//         q[j] = deCasteljauDx(std::span(p, sizeU), u);
//     }

//     // Normal not normalized.
//     return deCasteljau(std::span(q, sizeV), v);
// }

// vec4f derivativeV(const PatchRef<vec4f,uint32_t> &s, float u, float v)
// {
//     vec4f p[MAX_POINTS];
//     vec4f q[MAX_POINTS];

//     auto sizeU = s.shape(0);
//     auto sizeV = s.shape(1);
//     assert(sizeU <= MAX_POINTS && sizeV <= MAX_POINTS);

//     for (auto i = 0; i < sizeU; i++)
//     {
//         for (auto j = 0; j < sizeV; j++)
//             q[j] = s.point(i, j);
//         p[i] = deCasteljauDx(std::span(q, sizeV), v);
//     }

//     // Normal not normalized.
//     return deCasteljau(std::span(p, sizeU), u);
// }

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
