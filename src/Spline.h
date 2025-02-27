#pragma once

#include <geometry/Bounds3.h>
#include <geometry/Index3.h>
#include <geometry/Intersection.h>
#include <cuda/std/concepts>
#include "AlignedTypes.h"
#include "ArrayAdaptor.h"
#include "rt/Intersection.h"

#define _SPL_DISABLED_WHILE_RANGES_NOT_IMPLEMENTED_FOR_LIBCUXX

#define SPL_DERIVATIVE_MAXPOINTS 8U
#define SPL_NORMAL_FIXVALUE 0.0001f
#define SPL_MAX_LOCAL_ARRAY_SIZE 16U

// Enable this do enable Bézier Clipping statistics
// @deprecated do not enable
// #define SPL_BC_STATS

// Except where noted, the patches are assumed to be bicubic Bézier patches.

namespace cg::spline
{

template<typename V, typename C>
concept is_vector_component_type =
    std::same_as<C, std::remove_cvref_t<typename V::value_type>> ||
    std::same_as<C, std::remove_cvref_t<V>>;

template<typename T>
concept surface =
requires (const T s, typename T::value_type u, typename T::value_type v, int i)
{
    { s(u,v) } -> std::same_as<typename T::point_type>;
    { s.normal(u,v) } -> std::same_as<typename T::point_type>;
    { s.derivative(u,v) } -> std::same_as<typename T::point_type>;
    { s.bounds() } -> std::convertible_to<Bounds3<typename T::value_type>>;
    { s.domain() } -> std::convertible_to<Bounds2<typename T::value_type>>;
    { s.degree() } -> std::convertible_to<Index2<int>>;
};

template<typename T>
concept surface_cage = requires (const T s, int i, int j)
{
    { s[i] } -> std::same_as<const typename T::point_type&>;
    { s.point(i,j) } -> std::same_as<const typename T::point_type&>;
    { s.shape(i) } -> std::integral;
    { s.size() } -> std::integral;
    requires std::copy_constructible<typename T::point_type>;
    requires requires (T t)
    {
        { t[i] } -> std::same_as<typename T::point_type&>;
        { t.point(i,j) } -> std::same_as<typename T::point_type&>;
    };
};

/// Reference to a bicubic Bézier patch.
template<typename vec>
class Patch
{
public:
    using point_type = vec;

    _SPL_CONSTEXPR explicit
    Patch(vec* controlPoints) : _points{controlPoints} {}

    _SPL_CONSTEXPR Patch() = default;
    _SPL_CONSTEXPR Patch(Patch&&) = default;
    _SPL_CONSTEXPR Patch(const Patch&) = default;

    _SPL_CONSTEXPR Patch& operator= (Patch&&) = default;
    _SPL_CONSTEXPR Patch& operator= (const Patch&) = default;

    _SPL_CONSTEXPR
    const vec& operator[] (uint32_t i) const { return _points[i]; }

    _SPL_CONSTEXPR
    vec& operator[] (uint32_t i) { return _points[i]; }

    _SPL_CONSTEXPR
    const vec& point(uint32_t i, uint32_t j) const { return (*this)[4*j + i]; }

    _SPL_CONSTEXPR
    vec& point(uint32_t i, uint32_t j) { return (*this)[4*j + i]; }

    _SPL_CONSTEXPR uint32_t shape(uint32_t i) const noexcept
    {
        return i < 2 ? 4 : 0;
    }

    _SPL_CONSTEXPR uint32_t size() const noexcept { return 16; }

protected:
    vec* _points {};
};

/**
 * Reference to a bicubic Bézier patch with control points located anywhere
 * inside a Vertex/Point buffer/array. The addresses of the points are not
 * assumed to be in a sequence, so it needed to provide an index array, that
 * references the control points, representing the patch.
 * 
 * @tparam vec Control point array element type 
 * @tparam idx Index array element type
 */
template<typename vec, typename idx>
class PatchRef
{
public:
    // TODO Breaking change to support more use cases:
    //   using point_type = std::ranges::range_value_t<vec>;
    // in case of [vec = vec3f*] or [vec = std::array<vec3f, 16>].
    // Maybe rename vec.

    using point_type = vec;
    using index_type = idx;

    _SPL_CONSTEXPR explicit
    PatchRef(vec* pointBuffer, idx* patchIndexes)
        : _points{pointBuffer}, _indexes{patchIndexes} {}

    _SPL_CONSTEXPR explicit
    PatchRef(vec* pointBuffer, idx* indexBuffer, idx patchNumber)
        : PatchRef(pointBuffer, indexBuffer + 16*patchNumber) {}

    _SPL_CONSTEXPR PatchRef() = default;
    _SPL_CONSTEXPR PatchRef(PatchRef&&) = default;
    _SPL_CONSTEXPR PatchRef(const PatchRef&) = default;

    _SPL_CONSTEXPR
    const vec& operator[] (uint32_t i) const { return _points[_indexes[i]]; }

    _SPL_CONSTEXPR
    vec& operator[] (uint32_t i) { return _points[_indexes[i]]; }

    _SPL_CONSTEXPR
    const vec& point(uint32_t i, uint32_t j) const { return (*this)[4*j + i]; }

    _SPL_CONSTEXPR
    vec& point(uint32_t i, uint32_t j) { return (*this)[4*j + i]; }

    _SPL_CONSTEXPR uint32_t shape(uint32_t i) const noexcept
    {
        return i < 2 ? 4 : 0;
    }

    _SPL_CONSTEXPR uint32_t size() const noexcept { return 16; }

#ifndef _SPL_DISABLED_WHILE_RANGES_NOT_IMPLEMENTED_FOR_LIBCUXX
    auto asRange() const
    {
        return std::views::transform(
            std::views::counted(_indexes, 16),
            [&](idx i) -> const vec& { return _points[i]; }
        );
    }
#endif

protected:
    vec* _points {};
    idx* _indexes {};
};

/**
 * De Casteljau algorithm for computing a cubic Bézier curve.
 */
template<typename vec, typename real = typename vec::value_type>
_SPL_CONSTEXPR vec& deCasteljau(vec p[4], real u)
{
    // deCasteljau algorithm (The NURBS Book, page 24)
    const real t = 1 - u;

    for (unsigned k = 1; k < 4; ++k)
        for (unsigned i = 0; i < 4 - k; ++i)
            p[i] = t * p[i] + u * p[i + 1];
    return p[0];
}

/**
 * De Casteljau algorithm for computing a cubic Bézier curve derivative.
 */
template<typename vec, typename real = typename vec::value_type>
_SPL_CONSTEXPR vec& deCasteljauDx(vec p[4], real u)
{
    const real t = 1 - u;

    for (unsigned i = 1; i < 4; ++i)
        p[i - 1] = p[i] - p[i - 1];

    for (unsigned k = 2; k < 4; ++k)
        for (unsigned i = 0; i < 4 - k; ++i)
            p[i] = t * p[i] + u * p[i + 1];
    return p[0];
}

template<surface_cage S, typename real = S::point_type::value_type>
_SPL_CONSTEXPR
auto interpolate(const S& s, real u, real v) ->
    std::remove_cvref_t<typename S::point_type>
{
    using vec = std::remove_cvref_t<typename S::point_type>;
    vec p[4];
    vec q[4];

    const real t = 1 - u;
    for (unsigned j = 0; j < 4; j++)
    {
        for (unsigned i = 0; i < 4; ++i)
            p[i] = s.point(i, j);
        q[j] = deCasteljau(p, u);
    }
    return deCasteljau(q, v);
}

template<surface_cage S, typename real = S::point_type::value_type>
_SPL_CONSTEXPR
auto derivativeU(const S& s, real u, real v) ->
    std::remove_cvref_t<typename S::point_type>
{
    using vec = std::remove_cvref_t<typename S::point_type>;
    vec p[4];
    vec q[4];

    for (auto j = 0; j < 4; j++)
    {
        for (auto i = 0; i < 4; i++)
            p[i] = s.point(i, j);
        q[j] = deCasteljauDx(p, u);
    }

    // Normal not normalized.
    return deCasteljau(q, v);
}

template<surface_cage S, typename real = S::point_type::value_type>
_SPL_CONSTEXPR
auto derivativeV(const S& s, real u, real v) ->
    std::remove_cvref_t<typename S::point_type>
{
    using vec = std::remove_cvref_t<typename S::point_type>;
    vec p[4];
    vec q[4];

    for (auto i = 0; i < 4; i++)
    {
        for (auto j = 0; j < 4; j++)
            q[j] = s.point(i, j);
        p[i] = deCasteljauDx(q, v);
    }

    // Normal not normalized.
    return deCasteljau(p, u);
}

/**
 * Compute Bézier surface normal at coordinates ( @a u , @a v ).
 * @warning Not tested with rational Bézier surfaces. Be careful!
 * @warning Normal not normalized.
 */
HOST DEVICE vec3 normal(const PatchRef<vec4,uint32_t> &s, float u, float v);

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
requires is_vector_component_type<vec,real>
_SPL_CONSTEXPR
void split0(vec C[4], real t)
{
    assert(0.0 <= t && t <= 1.0);

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
requires is_vector_component_type<vec,real>
_SPL_CONSTEXPR
void split1(vec C[4], real t)
{
    assert(0.0 <= t && t <= 1.0);

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
requires is_vector_component_type<vec,real> && (_stride > 0)
_SPL_CONSTEXPR
void slice(vec *curve, real u, real v)
{
    assert(u <= v);
    // if (u > v) std::swap(u, v);

    // `stride_view` is only present in C++23
    auto C = [&](uint32_t i) -> vec& { return curve[i*_stride]; };

    real iu = 1 - u;
    real vv = (v - u) / iu;
    real ivv = 1 - vv;
    vec P5, P6, P8;

    // split1(C, u);
    // if (u > 0)
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
    // if (vv < 1)
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
// requires is_vector_component_type<vec,real>
_SPL_CONSTEXPR
void subpatchU(vec *patch, real umin, real umax)
{
    for (int i = 0; i < 4; i++)
        slice(patch + 4*i, umin, umax);
}

template<typename vec, typename real>
// requires is_vector_component_type<vec,real>
_SPL_CONSTEXPR
void subpatchV(vec *patch, real vmin, real vmax)
{
    for (int i = 0; i < 4; i++)
        slice<4>(patch + i, vmin, vmax);
}

template<typename vec, typename real>
// requires is_vector_component_type<vec,real>
_SPL_CONSTEXPR
void subpatch(
    vec patch[16],
    real umin,
    real umax,
    real vmin,
    real vmax)
{
    subpatchV(patch, vmin, vmax);
    subpatchU(patch, umin, umax);
}

template<std::forward_iterator It>
HOST DEVICE
auto boundingbox(It begin, It end) //-> Bounds3<typename std::iter_value_t<It>::value_type>
{
    using vec = std::iter_value_t<It>;
    using real = typename vec::value_type;

    Bounds3<real> bounds {};

    for (auto it = begin; it != end; it++)
    {
        const auto& p = *it;
        if constexpr (requires (vec v) { v.w; }) // does vector component w exist?
            bounds.inflate(project(p));
        else
            bounds.inflate(p);
    }

    return bounds;
}

template<typename real, std::forward_iterator It, typename Fn>
HOST DEVICE
auto boundingbox(It begin, It end, Fn unaryop)
{
    Bounds3<real> bounds {};

    for (auto it = begin; it != end; it++)
    {
        bounds.inflate(unaryop(*it));
    }

    return bounds;
}

template<typename vec, typename idx>
HOST DEVICE
auto boundingbox(const vec* buffer, const idx patch[16])
{
    using real = typename vec::value_type;

    Bounds3<real> bounds {};

    for (auto i = 0U; i < 16; i++)
    {
        const auto& p = buffer[patch[i]];
        if constexpr (requires (vec v) { v.w; })
            bounds.inflate(project(p));
        else
            bounds.inflate(p);
    }

    return bounds;
}

#ifndef _SPL_DISABLED_WHILE_RANGES_NOT_IMPLEMENTED_FOR_LIBCUXX
template<std::ranges::forward_range V>
inline
auto boundingbox(V v) -> Bounds3<typename std::ranges::range_value_t<V>::value_type>
{
    return boundingbox(std::begin(v), std::end(v));
}
#endif

template<typename vec, typename real, typename idx>
HOST DEVICE
Bounds3<real> subpatchBoundingbox(
    vec subp[16],
    const vec* buffer,
    const idx patch[16],
    real umin,
    real umax,
    real vmin,
    real vmax)
{
    for (auto i = 0u; i < 16u; i++)
        subp[i] = buffer[patch[i]];

    subpatch(subp, umin, umax, vmin, vmax);
    return boundingbox(subp, subp+16);
}

// HOST DEVICE
// bool doBezierClipping(rt::Intersection& hit,
//     const rt::Ray& ray,
//     const vec4f buffer[],
//     const uint32_t patch[16],
//     float tol = 0x1p-12f);

// Ray represented only the direction vector
HOST DEVICE
bool doBezierClipping(Intersection& hit,
    const Ray3f& ray,
    const vec4 buffer[],
    const uint32_t patch[16],
    float tol = 0x1p-12f);

HOST DEVICE
bool doSubdivision(Intersection& hit,
    const Ray3f& ray,
    const vec4 buffer[],
    const uint32_t patch[16],
    float tol = 0x1p-4f);


DEVICE
bool doSubdivision_device(Intersection& hit,
    const Ray3f& ray,
    const vec4 buffer[],
    const uint32_t patch[16],
    float tol = 0x1p-12f);

/**
 * Generic (template) definitions that deals with higher and arbitrary
 * surface degrees.
 */
namespace ext
{

static constexpr auto MAX_POINTS = SPL_DERIVATIVE_MAXPOINTS;

#ifndef _SPL_DISABLED_WHILE_RANGES_NOT_IMPLEMENTED_FOR_LIBCUXX
template<std::ranges::random_access_range V,
    typename real = std::ranges::range_value_t<V>::value_type>
auto& deCasteljau(V p, real u)
{
    // deCasteljau algorithm (The NURBS Book, page 24)
    const real t = 1 - u;

    unsigned size = p.size();
    for (unsigned k = 1; k < size; ++k)
        for (unsigned i = 0; i < size - k; ++i)
            p[i] = t * p[i] + u * p[i + 1];
    return p[0];
}

template<std::ranges::random_access_range V,
    typename real = std::ranges::range_value_t<V>::value_type>
auto& deCasteljauDx(V p, real u)
{
    const real t = 1 - u;
    unsigned size = p.size();

    for (unsigned i = 1; i < size; ++i)
        p[i - 1] = p[i] - p[i - 1];

    for (unsigned k = 2; k < size; ++k)
        for (unsigned i = 0; i < size - k; ++i)
            p[i] = t * p[i] + u * p[i + 1];
    return p[0];
}
#endif

template<typename vec, typename real = typename vec::value_type>
_SPL_CONSTEXPR
vec& deCasteljau(unsigned size, vec p[], real u)
{
    // deCasteljau algorithm (The NURBS Book, page 24)
    const real t = 1 - u;

    for (unsigned k = 1; k < size; ++k)
        for (unsigned i = 0; i < size - k; ++i)
            p[i] = t * p[i] + u * p[i + 1];
    return p[0];
}

template<typename vec, typename real = typename vec::value_type>
_SPL_CONSTEXPR
vec& deCasteljauDx(unsigned size, vec p[], real u)
{
    const real t = 1 - u;

    for (unsigned i = 1; i < size; ++i)
        p[i - 1] = p[i] - p[i - 1];

    for (unsigned k = 2; k < size; ++k)
        for (unsigned i = 0; i < size - k; ++i)
            p[i] = t * p[i] + u * p[i + 1];
    return p[0];
}

template<surface_cage S, typename real>
requires is_vector_component_type<typename S::point_type, real>
_SPL_CONSTEXPR
auto interpolate(const S& s, real u, real v) ->
    std::remove_cvref_t<typename S::point_type>
{
    using vec = std::remove_cvref_t<typename S::point_type>;
    vec p[MAX_POINTS];
    vec q[MAX_POINTS];

    auto sizeU = s.shape(0);
    auto sizeV = s.shape(1);
    assert(sizeU <= MAX_POINTS && sizeV <= MAX_POINTS);

    const real t = 1 - u;
    for (int j = 0; j < sizeV; j++)
    {
        for (int i = 0; i < sizeU; ++i)
            p[i] = s.point(i, j);
        q[j] = deCasteljau(sizeU, p, u);
    }
    return deCasteljau(sizeV, q, v);
}

template<surface_cage S, typename real>
requires is_vector_component_type<typename S::point_type,real>
_SPL_CONSTEXPR
auto derivativeU(const S& s, real u, real v) ->
    std::remove_cvref_t<typename S::point_type>
{
    using vec = std::remove_cvref_t<typename S::point_type>;
    vec p[MAX_POINTS];
    vec q[MAX_POINTS];

    auto sizeU = s.shape(0);
    auto sizeV = s.shape(1);
    assert(sizeU <= MAX_POINTS && sizeV <= MAX_POINTS);

    for (auto j = 0; j < sizeV; j++)
    {
        for (auto i = 0; i < sizeU; i++)
            p[i] = s.point(i, j);
        q[j] = deCasteljauDx(sizeU, p, u);
    }

    // Normal not normalized.
    return deCasteljau(sizeV, q, v);
}

template<surface_cage S, typename real>
requires is_vector_component_type<typename S::point_type,real>
_SPL_CONSTEXPR
auto derivativeV(const S& s, real u, real v) ->
    std::remove_cvref_t<typename S::point_type>
{
    using vec = std::remove_cvref_t<typename S::point_type>;
    vec p[MAX_POINTS];
    vec q[MAX_POINTS];

    auto sizeU = s.shape(0);
    auto sizeV = s.shape(1);
    assert(sizeU <= MAX_POINTS && sizeV <= MAX_POINTS);

    for (auto i = 0; i < sizeU; i++)
    {
        for (auto j = 0; j < sizeV; j++)
            q[j] = s.point(i, j);
        p[i] = deCasteljauDx(sizeV, q, v);
    }

    // Normal not normalized.
    return deCasteljau(sizeU, p, u);
}

// template<surface_cage S, typename real>
// requires std::same_as<typename S::point_type, Vector4<real>>
// auto normal(const S& s, real u, real v) -> Vector4<real>
// {
//     constexpr real e = SPL_NORMAL_FIXVALUE;

//     auto derU = derivativeU(s, u, v);
//     auto derV = derivativeV(s, u, v);
//     // Fix for degenerated patches.
//     if (derU.x == 0 && derU.y == 0 && derU.z == 0)
//     {
//         derU = derivativeU(s, u, std::abs(v - e));
//     }
//     if (derV.x == 0 && derV.y == 0 && derV.z == 0)
//     {
//         derV = derivativeV(s, std::abs(u - e), v);
//     }
//     auto N = Vector3<real>(derU).cross(Vector3<real>(derV));
//     // Ignoring rational patches for now.
//     return Vector4<real>(N.x, N.y, N.z, 1.0);
// }

} // namespace ext

using ::cuda::std::numeric_limits;

/**
 * @brief Computes the distance between a point P and a plane represented by the
 *        normal vector N and origin O.
 * 
 * @param N The plane normal vector (must be unit length)
 * @param O The plane origin point
 * @param P The point whose distance is being measured
 * @return float The computed distance
 */
_SPL_CONSTEXPR
float distance(const vec3& N, const vec3& O, const vec3& P)
{
    return N.dot(P - O);
}

/**
 * @brief Computes the distance between a point P and a line represented by the
 *        normal vector N and origin O.
 * 
 * @param N The line normal vector (must be unit length)
 * @param O The line origin point
 * @param P The point whose distance is being measured
 * @return float The computed distance
 */
_SPL_CONSTEXPR
float distance(const vec2& N, const vec2& O, const vec2& P)
{
    return N.dot(P - O);
}

static HOST DEVICE
bool triangleOriginIntersection(vec2 &coord, vec2 P, vec2 A, vec2 B)
{
    vec2 p = A - P;
    vec2 q = B - P;
    float det = p.x*q.y - q.x*p.y;

    // `det` will be often small (< 1e-7) but not zero
    // and the intersection should not be discarded
    if (fabs(det) < 0x1p-80f) // ~ 1e-24f (not a subnormal number)
        return false;

    det = 1.0f / det;

    // 2x2 inversion matrix
    vec2 m0 = vec2( q.y, -q.x); // first line
    vec2 m1 = vec2(-p.y,  p.x); // second line

    auto u = det * m0.dot(-P);
    if (u < 0 || u > 1)
        return false;

    auto v = det * m1.dot(-P);
    if (v < 0 || u + v > 1)
        return false;

    coord.set(u, v);
    return true;
}

static HOST DEVICE
bool approximateIntersection(std::predicate<vec2> auto onHit,
    const Patch<vec2>& p, vec2 pmin, vec2 pmax, vec2 psize)
{
    bool found = false;
    // there are cases where is impossible to clip the patch because
    // it is already so small that it is a plane and the intersection
    // has been already found
    // thus, the infinite subdivision must be stopped

    // intersect triangles to avoid multiple intersection problem
    auto p00 = p.point(0,0);   // (u,v) = (0,0)
    auto p10 = p.point(3,0);   // (1,0)
    auto p01 = p.point(0,3);   // (0,1)
    auto p11 = p.point(3,3);   // (1,1)

    vec2 coord;
    if (triangleOriginIntersection(coord, p00, p10, p01))
    {
        found |= onHit(pmin + coord * psize);
    }
    if (triangleOriginIntersection(coord, p11, p01, p10))
    {
        found |= onHit(pmax - coord * psize);
    }
    return found;
}

constexpr HOST DEVICE
vec2 min(const vec2 a, const vec2 b)
{
    return {fmin(a.x, b.x), fmin(a.y, b.y)};
}

constexpr HOST DEVICE
vec2 max(const vec2 a, const vec2 b)
{
    return {fmax(a.x, b.x), fmax(a.y, b.y)};
}

// Used once as debug code
#ifdef SPL_BC_STATS
namespace stats
{
struct BCSearchStep
{
    vec2 L; // midline
    vec2 min;
    vec2 max;
    char cutside; // "U" or "V"
    float lower;
    float upper;
};

struct BCSearchHit
{
    float distance;
    vec2 coord;
};

struct BCData
{
    std::array<vec2,16> patch2D;
    std::vector<BCSearchStep> steps {};
    std::vector<BCSearchHit> hits {};
    int maxStackDepth = 0;
};

extern std::vector<BCData> g_BezierClippingData;
extern bool g_BezierClippingEnable;
} // namespace stats
#endif

} // namespace cg::spline
