#pragma once

#include "Spline.h"

namespace cg::spline::mat
{

constexpr mat4 CUBIC_BASE
{
    { 1,  0,  0, 0},
    {-3,  3,  0, 0},
    { 3, -6,  3, 0},
    {-1,  3, -3, 1},
};

constexpr mat4 CUBIC_DERIVATIVE_BASE
{
    {-3,   3,  0, 0},
    { 6, -12,  6, 0},
    {-3,   9, -9, 3},
    { 0,   0,  0, 0},
};

_SPL_CONSTEXPR vec4 _coef_vec(float t)
{
    float t2 = t*t;
    return {1, t, t2, t2*t};
}

_SPL_CONSTEXPR vec4 _coef_vec_dt(float t)
{
    return {1, t, t*t, 0};
}

_SPL_CONSTEXPR vec4 interpolate(const vec4 p[4], float u)
{
    vec4 t = _coef_vec(u);
    return mat4(p) * CUBIC_BASE * t;
}

// Compute a cubic Bézier curve derivative.
_SPL_CONSTEXPR vec4 derivative(const vec4 p[4], float u)
{
    vec4 t = _coef_vec_dt(u);
    return mat4(p) * CUBIC_DERIVATIVE_BASE * t;
}

template<surface_cage S>
requires std::same_as<vec4, std::remove_cvref_t<typename S::point_type>>
_SPL_CONSTEXPR vec4 interpolate(const S& s, float u, float v)
{
    constexpr auto B = CUBIC_BASE;
    vec4 x = B * _coef_vec(u);
    vec4 y = B * _coef_vec(v);
    mat4 A
    {
        mat4(s[ 0],s[ 1],s[ 2],s[ 3]) * x,
        mat4(s[ 4],s[ 5],s[ 6],s[ 7]) * x,
        mat4(s[ 8],s[ 9],s[10],s[11]) * x,
        mat4(s[12],s[13],s[14],s[15]) * x
    };
    return A * y;
}

template<surface_cage S>
requires std::same_as<vec4, std::remove_cvref_t<typename S::point_type>>
_SPL_CONSTEXPR vec4 derivativeU(const S& s, float u, float v)
{
    constexpr auto B = CUBIC_BASE;
    constexpr auto C = CUBIC_DERIVATIVE_BASE;
    vec4 x = C * _coef_vec_dt(u);
    vec4 y = B * _coef_vec(v);
    mat4 A
    {
        mat4(s[ 0],s[ 1],s[ 2],s[ 3]) * x,
        mat4(s[ 4],s[ 5],s[ 6],s[ 7]) * x,
        mat4(s[ 8],s[ 9],s[10],s[11]) * x,
        mat4(s[12],s[13],s[14],s[15]) * x
    };
    return A * y;
}

template<surface_cage S>
requires std::same_as<vec4, std::remove_cvref_t<typename S::point_type>>
_SPL_CONSTEXPR vec4 derivativeV(const S& s, float u, float v)
{
    constexpr auto B = CUBIC_BASE;
    constexpr auto C = CUBIC_DERIVATIVE_BASE;
    vec4 x = B * _coef_vec(u);
    vec4 y = C * _coef_vec_dt(v);
    mat4 A
    {
        mat4(s[0],s[4],s[ 8],s[12]) * y,
        mat4(s[1],s[5],s[ 9],s[13]) * y,
        mat4(s[2],s[6],s[10],s[14]) * y,
        mat4(s[3],s[7],s[11],s[15]) * y
    };
    return A * x;
}

template<surface_cage S>
_SPL_CONSTEXPR vec3 normal(const S& s, float u, float v)
{
    // constexpr float e = SPL_NORMAL_FIXVALUE;
    // u = fabs(u - e);
    // v = fabs(v - e);
    vec4 dU = derivativeU(s, u, v);
    vec4 dV = derivativeV(s, u, v);
    return vec3(dU).cross(vec3(dV));
}

} // namespace cg::spline::mat
