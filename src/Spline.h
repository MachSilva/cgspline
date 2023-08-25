#pragma once

#include <geometry/Bounds3.h>
#include <geometry/Index3.h>
#include <geometry/Intersection.h>
#include <algorithm>
#include <concepts>
#include <cstdint>
#include <ranges>
#include <span>
#include <stack>

namespace cg::spline
{

template<typename V, typename C>
concept vector_component_type_check = std::same_as<C, typename V::value_type>;

template<typename T>
concept surface = requires (const T s, typename T::value_type u, typename T::value_type v, int i)
{
    { s(u,v) } -> std::same_as<typename T::point_type>;
    { s.normal(u,v) } -> std::same_as<typename T::point_type>;
    { s.derivative(u,v) } -> std::same_as<typename T::point_type>;
    { s.bounds() } -> std::convertible_to<Bounds3<typename T::value_type>>;
    { s.domain() } -> std::convertible_to<Bounds2<typename T::value_type>>;
    { s.degree() } -> std::convertible_to<Index2<int>>;
};

template<typename T>
concept surface_cage = requires (T s, int i, int j)
{
    { s[i] } -> std::same_as<typename T::point_type&>;
    { s.point(i,j) } -> std::same_as<typename T::point_type&>;
    { s.shape(i) } -> std::integral;
    { s.size() } -> std::integral;
};

template<std::ranges::random_access_range V,
    typename real = std::ranges::range_value_t<V>::value_type>
auto& deCasteljau(V p, real u)
{
    // deCasteljau algorithm (The NURBS Book, page 24)
    const real t = 1 - u;

    uint32_t size = p.size();
    for (uint32_t k = 1; k < size; ++k)
        for (uint32_t i = 0; i < size - k; ++i)
            p[i] = t * p[i] + u * p[i + 1];
    return p[0];
}

/**
 * Cubic bézier curve splitting at point t.
 * 
 * Let C0 and C1 be two curves derived from an original curve C = B(P0,P1,P2,P3),
 * where C0 represents the curve in the interval [0,t] of C and C1 is represents
 * the curve in the interval [t,1].
 * 
 * Compute the following points:
 * 
 *   P4 = (1-t)P0 + tP1
 *   P5 = (1-t)P1 + tP2
 *   P6 = (1-t)P2 + tP3
 * 
 *   P7 = (1-t)P4 + tP5
 *   P8 = (1-t)P5 + tP6
 * 
 *   P9 = (1-t)P7 + tP8
 * 
 * Therefore, C0 = B(P0,P4,P7,P9) and C1 = B(P9,P8,P6,P3).
 */

// Curve equivalent: [0, t]
template<typename vec, typename real>
requires vector_component_type_check<vec,real> 
void split0(vec C[4], real t)
{
    const real it = 1 - t;
    vec P5, P6, P8;

    // C[0] = C[0];
    P6   = it * C[2] + t * C[3];
    P5   = it * C[1] + t * C[2];
    C[1] = it * C[0] + t * C[1]; // C[1] = P4
    P8   = it *   P5 + t *   P6;
    C[2] = it * C[1] + t *   P5; // C[2] = P7
    C[3] = it * C[2] + t *   P8; // C[3] = P9
}

// Curve equivalent: [t, 1]
template<typename vec, typename real>
requires vector_component_type_check<vec,real> 
void split1(vec C[4], real t)
{
    const real it = 1 - t;
    vec P5, P6, P8;

    // C[3] = C[3];
    P6   = t * C[1] + it * C[0];
    P5   = t * C[2] + it * C[1];
    C[2] = t * C[3] + it * C[2]; // C[2] = P4
    P8   = t *   P5 + it *   P6;
    C[1] = t * C[2] + it *   P5; // C[1] = P7
    C[0] = t * C[1] + it *   P8; // C[0] = P9
}

// Curve equivalent: [u, v]
template<uint32_t _stride = 1, typename vec, typename real>
requires vector_component_type_check<vec,real> && (_stride > 0)
void slice(vec *curve, real u, real v)
{
    if (u > v) std::swap(u, v);

    // `stride_view` is only present in C++23
    auto C = [&](uint32_t i) -> vec& { return curve[i*_stride]; };

    real iu = 1 - u;
    real vv = (v - u) / iu;
    real ivv = 1 - vv;
    vec P5, P6, P8;

    // split1(C, u);
    if (u > 0)
    {
        // C(3) = C(3);
        P6   = u * C(1) + iu * C(0);
        P5   = u * C(2) + iu * C(1);
        C(2) = u * C(3) + iu * C(2); // C(2) = P4
        P8   = u *   P5 + iu *   P6;
        C(1) = u * C(2) + iu *   P5; // C(1) = P7
        C(0) = u * C(1) + iu *   P8; // C(0) = P9
    }

    // split0(C, vv);
    if (vv < 1)
    {
        // C(0) = C(0);
        P6   = ivv * C(2) + vv * C(3);
        P5   = ivv * C(1) + vv * C(2);
        C(1) = ivv * C(0) + vv * C(1); // C(1) = P4
        P8   = ivv *   P5 + vv *   P6;
        C(2) = ivv * C(1) + vv *   P5; // C(2) = P7
        C(3) = ivv * C(2) + vv *   P8; // C(3) = P9
    }
}

template<typename vec, typename real>
// requires vector_component_type_check<vec,real>
void subpatch(
    vec patch[16],
    real umin,
    real umax,
    real vmin,
    real vmax)
{
    for (int i = 0; i < 4; i++)
        slice<4>(patch + i, vmin, vmax);
    for (int i = 0; i < 4; i++)
        slice(patch + 4*i, umin, umax);
}

template<std::forward_iterator It>
auto boundingbox(It begin, It end) //-> Bounds3<typename std::iter_value_t<It>::value_type>
{
    using vec = std::iter_value_t<It>;
    using real = typename vec::value_type;

    Bounds3<real> bounds {};

    for (auto it = begin; it != end; it++)
    {
        const auto& p = *it;
        if constexpr (requires (vec v) { v.w; }) // does vector component w exist?
            bounds.inflate(p.x / p.w, p.y / p.w, p.z / p.w);
        else
            bounds.inflate(p);
    }

    return bounds;
}

template<std::ranges::forward_range V>
inline
auto boundingbox(V v) -> Bounds3<typename std::ranges::range_value_t<V>::value_type>
{
    return boundingbox(std::begin(v), std::end(v));
}

template<typename vec, typename idx>
auto boundingbox(const vec* buffer, const idx patch[16])
{
    return boundingbox(std::views::transform(
        std::views::counted(patch, 16),
        [&](idx i) -> const vec& { return buffer[i]; }
    ));
}

template<typename vec, typename real, typename idx>
Bounds3<real> subpatchBoundingbox(
    const vec* buffer,
    const idx patch[16],
    real umin,
    real umax,
    real vmin,
    real vmax)
{
    vec sub[16];
    for (auto i = 0u; i < 16u; i++)
        sub[i] = buffer[patch[i]];

    subpatch(sub, umin, umax, vmin, vmax);
    return boundingbox(std::begin(sub), std::end(sub));
}

template<typename T>
bool doBezierClipping(Intersection& hit, const Ray3f& ray, T patch)
{
    return false;
}

bool doSubdivision(Intersection& hit,
    const Ray3f& ray,
    const vec4f buffer[],
    const uint32_t patch[16],
    float threshold = 0.005f);

} // namespace cg::spline

