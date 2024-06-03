#pragma once

#include "Spline.h"

namespace cg::spline::mat
{

/**
 * @brief Bézier base function coefficients in a matrix.
 * 
 * `Pi` means control point number i.
 * `Mᵀ` means the transpose of the matrix M.
 * 
 * Let D be the number of dimensions of control points:
 * - P0 is a Dx1 (column) matrix or a vector with D dimensions;
 * - [P0 P1 P2 P3] is a Dx4 matrix;
 * - [P0 P1 P2 P3]ᵀ is a 4xD matrix.
 * 
 * How to use: B(t) = [P0 P1 P2 P3] * CUBIC_BASE * [1 t t² t³]ᵀ
 *         or: B(t) = [1 t t² t³] * CUBIC_BASEᵀ * [P0 P1 P2 P3]ᵀ
 */
_SPL_CONSTEXPR mat4 CUBIC_BASE
{
    { 1,  0,  0, 0},
    {-3,  3,  0, 0},
    { 3, -6,  3, 0},
    {-1,  3, -3, 1},
};

_SPL_CONSTEXPR mat4 CUBIC_BASE_INVERSE
{
    {1, 0, 0, 0},
    {1, 1.0f/3.0f, 0, 0},
    {1, 2.0f/3.0f, 1.0f/3.0f, 0},
    {1, 1, 1, 1},
};

_SPL_CONSTEXPR mat4 CUBIC_DERIVATIVE_BASE
{
    {-3,   3,  0, 0},
    { 6, -12,  6, 0},
    {-3,   9, -9, 3},
    { 0,   0,  0, 0},
};

_SPL_CONSTEXPR mat4 split_matrix(float t)
{
#ifdef __CUDA_ARCH__
    __builtin_assume (t >= 0.0f && t <= 1.0f);
#endif
    float s = 1 - t;
    float s2 = s * s;
    float t2 = t * t;
    return
    { // columns
        {   1,      0,      0,    0},
        {   s,      t,      0,    0},
        {  s2,  2*s*t,     t2,    0},
        {s2*s, 3*s2*t, 3*s*t2, t2*t},
    };
}

_SPL_CONSTEXPR mat4 slice_matrix(float a, float b)
{
    const auto
        a2 = a*a,
        b2 = b*b,
        ia = 1 - a,
        ib = 1 - b,
        ia2 = ia*ia,
        ib2 = ib*ib,
        _a_2b_3ab = a + 2*b - 3*a*b,
        _2a_b_3ab = 2*a + b - 3*a*b;
    return
    {
        {ia2*ia, 3*a*ia2, 3*a2*ia, a2*a},
        {ia2*ib, ia*_2a_b_3ab, a*_a_2b_3ab, a2*b},
        {ia*ib2, ib*_a_2b_3ab, b*_2a_b_3ab, a*b2},
        {ib2*ib, 3*b*ib2, 3*b2*ib, b2*b},
    };
}

// Original definition
// _SPL_CONSTEXPR mat4 slice_matrix(float a, float b)
// {
//     const auto
//         a2 = a*a,
//         ba = b - a,
//         ba2 = ba*ba;
//     const mat4 Q
//     {
//         {1,  a,     a2,    a2*a},
//         {0, ba, 2*a*ba, 3*a2*ba},
//         {0,  0,    ba2, 3*a*ba2},
//         {0,  0,      0,  ba2*ba},
//     };
//     return CUBIC_BASE * Q * CUBIC_BASE_INVERSE;
// }

_SPL_CONSTEXPR vec4 _eval_vec(float t)
{
    float t2 = t*t;
    return {1, t, t2, t2*t};
}

_SPL_CONSTEXPR vec4 _eval_vec_dt(float t)
{
    return {1, t, t*t, 0};
}

_SPL_CONSTEXPR vec4 interpolate(const vec4 p[4], float u)
{
    vec4 t = _eval_vec(u);
    return mat4(p) * CUBIC_BASE * t;
}

// Compute a cubic Bézier curve derivative.
_SPL_CONSTEXPR vec4 derivative(const vec4 p[4], float u)
{
    vec4 t = _eval_vec_dt(u);
    return mat4(p) * CUBIC_DERIVATIVE_BASE * t;
}

template<surface_cage S>
requires std::same_as<vec4, std::remove_cvref_t<typename S::point_type>>
_SPL_CONSTEXPR vec4 interpolate(const S& s, float u, float v)
{
    constexpr auto B = CUBIC_BASE;
    vec4 x = B * _eval_vec(u);
    vec4 y = B * _eval_vec(v);
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
    vec4 x = C * _eval_vec_dt(u);
    vec4 y = B * _eval_vec(v);
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
    vec4 x = B * _eval_vec(u);
    vec4 y = C * _eval_vec_dt(v);
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

// Curve equivalent: [u, v]
template<int stride = 1>
_SPL_CONSTEXPR void slice(vec2 *curve, float u, float v)
{
    // assert(u <= v);
    auto P = [&](int i) -> vec2& { return curve[i*stride]; };

    vec4 x (P(0).x, P(1).x, P(2).x, P(3).x);
    vec4 y (P(0).y, P(1).y, P(2).y, P(3).y);
    // x = V * U * x; // x^T * M = M^T * x
    mat4 M = slice_matrix(u, v).transposed();
    x = M * x;
    y = M * y;
    P(0) = {x[0], y[0]};
    P(1) = {x[1], y[1]};
    P(2) = {x[2], y[2]};
    P(3) = {x[3], y[3]};
}

// Curve equivalent: [u, v]
template<int stride = 1>
_SPL_CONSTEXPR void slice(vec4 *curve, float u, float v)
{
    auto& P0 = curve[0];
    auto& P1 = curve[stride];
    auto& P2 = curve[2*stride];
    auto& P3 = curve[3*stride];
    mat4 M (P0, P1, P2, P3);
    M = M * slice_matrix(u, v);
    P0 = M[0], P1 = M[1], P2 = M[2], P3 = M[3];
}

template<typename vec>
_SPL_CONSTEXPR void subpatchU(vec *patch, float umin, float umax)
{
    for (int i = 0; i < 4; i++)
        mat::slice(patch + 4*i, umin, umax);
}

template<typename vec>
_SPL_CONSTEXPR void subpatchV(vec *patch, float vmin, float vmax)
{
    for (int i = 0; i < 4; i++)
        mat::slice<4>(patch + i, vmin, vmax);
}

template<typename vec>
_SPL_CONSTEXPR
void subpatch(
    vec patch[16],
    float umin,
    float umax,
    float vmin,
    float vmax)
{
    subpatchV(patch, vmin, vmax);
    subpatchU(patch, umin, umax);
}

} // namespace cg::spline::mat
