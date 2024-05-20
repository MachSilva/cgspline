#pragma once

#include <cuda_runtime.h>
#include <math/Matrix4x4.h>

#define _SPL_CONSTEXPR __host__ __device__ constexpr

namespace cg::spline
{

struct vec2;
struct vec3;
struct vec4;

struct alignas(8) vec2 : Vector<float,2>
{
    using Vector::Vector;

    _SPL_CONSTEXPR vec2(const Vector& v) : Vector(v.x, v.y) {}
    // _SPL_CONSTEXPR vec2& operator =(const Vector& v) { x = v.x, y = v.y; return *this; }
    _SPL_CONSTEXPR operator float2() const { return {x, y}; }
};

struct alignas(16) vec3 : Vector<float,3>
{
    using Vector::Vector;

    _SPL_CONSTEXPR vec3(const vec2& v, float _z) : Vector(v.x, v.y, _z) {}

    _SPL_CONSTEXPR vec3(const Vector& v) : Vector(v.x, v.y, v.z) {}
    // _SPL_CONSTEXPR vec3& operator =(const Vector& v) { x = v.x, y = v.y, z = v.z; return *this; }
    _SPL_CONSTEXPR operator float3() const { return {x, y, z}; }
};

struct alignas(16) vec4 : Vector<float,4>
{
    using Vector::Vector;

    _SPL_CONSTEXPR vec4(const vec2& v, float _z, float _w) : Vector(v.x, v.y, _z, _w) {}
    _SPL_CONSTEXPR vec4(const vec3& v, float _w) : Vector(v.x, v.y, v.z, _w) {}

    _SPL_CONSTEXPR vec4(const Vector& v) : Vector(v) {}
    // _SPL_CONSTEXPR vec4& operator =(const Vector& v) { x = v.x, y = v.y, z = v.z, w = v.w; return *this; }

    _SPL_CONSTEXPR vec4& operator =(const float4& v) { x = v.x, y = v.y, z = v.z, w = v.w; return *this; }
    _SPL_CONSTEXPR operator float4() const { return {x, y, z, w}; }
};

_SPL_CONSTEXPR vec3 project(const vec4 &p)
{
    return {p.x / p.w, p.y / p.w, p.z / p.w};
}

struct alignas(16) quat : Quaternion<float>
{
    using Quaternion::Quaternion;

    _SPL_CONSTEXPR quat(const Quaternion& q) : Quaternion(q) {}
    // _SPL_CONSTEXPR quat& operator =(const Quaternion& q) { x = q.x, y = q.y, z = q.z, w = q.w; return *this; }
    _SPL_CONSTEXPR operator float4() const { return {x, y, z, w}; }
};

struct alignas(16) mat3
{
    vec3 column[3];

    _SPL_CONSTEXPR mat3() = default;

    _SPL_CONSTEXPR mat3(float d) : column{{1,0,0}, {0,1,0}, {0,0,1}} {}

    _SPL_CONSTEXPR mat3(const float* v)
        : column{{v[0],v[3],v[6]}, {v[1],v[4],v[7]}, {v[2],v[5],v[8]}} {}

    _SPL_CONSTEXPR mat3(vec3 v[3]) : column{v[0], v[1], v[2]} {}

    _SPL_CONSTEXPR mat3(const vec3& c0, const vec3& c1, const vec3& c2)
        : column{c0, c1, c2} {}

    _SPL_CONSTEXPR mat3(quat q)
    {
        // assert(fabs(q.length() - 1) < 1e-7);
        auto [i, j, k, r] = q;
        auto ii = 2 * i * i;
        auto ij = 2 * i * j;
        auto ik = 2 * i * k;
        auto jj = 2 * j * j;
        auto jk = 2 * j * k;
        auto kk = 2 * k * k;
        auto ri = 2 * r * i;
        auto rj = 2 * r * j;
        auto rk = 2 * r * k;
        column[0] = {1 - jj - kk, ij + rk, ik - rj};
        column[1] = {ij - rk, 1 - ii - kk, jk + ri};
        column[2] = {ik + rj, jk - ri, 1 - ii - jj};
    }

    _SPL_CONSTEXPR mat3(const mat3f& m) : column{m[0], m[1], m[2]} {}

    _SPL_CONSTEXPR vec3& operator [](int i) { return column[i]; }

    _SPL_CONSTEXPR const vec3& operator [](int i) const { return column[i]; }

    _SPL_CONSTEXPR float& operator ()(int i, int j) { return column[j][i]; }

    _SPL_CONSTEXPR float operator ()(int i, int j) const { return column[j][i]; }

    _SPL_CONSTEXPR vec3 operator *(const vec3& v) const
    {
        return v.x * column[0] + v.y * column[1] + v.z * column[2];
    }

    _SPL_CONSTEXPR mat3 operator *(const mat3& m) const
    {
        return {(*this) * m[0], (*this) * m[1], (*this) * m[2]};
    }

    _SPL_CONSTEXPR vec3 transform(const vec3& v) const
    {
        return (*this) * v;
    }

    _SPL_CONSTEXPR vec3 transformTransposed(const vec3& v) const
    {
        return { column[0].dot(v), column[1].dot(v), column[2].dot(v) };
    }

    _SPL_CONSTEXPR mat3 transposed() const
    {
        auto& [c0, c1, c2] = column;
        return {{c0.x, c1.x, c2.x}, {c0.y, c1.y, c2.y}, {c0.z, c1.z, c2.z}};
    }

    // _SPL_CONSTEXPR float determinant() const { ; }
};

struct alignas(16) mat4
{
    vec4 column[4];

    _SPL_CONSTEXPR mat4() = default;

    _SPL_CONSTEXPR mat4(float d)
        : column{{d,0,0,0}, {0,d,0,0}, {0,0,d,0}, {0,0,0,d}} {}

    /// Reads @a v as a row-major contiguous 4x4 matrix.
    // _SPL_CONSTEXPR mat4(const float* v)
    //     : column{{v[0],v[4],v[8],v[12]}, {v[1],v[5],v[9],v[13]}
    //     , {v[2],v[6],v[10],v[14]}, {v[3],v[7],v[11],v[15]}} {}

    _SPL_CONSTEXPR mat4(const vec4 v[4]) : column{v[0], v[1], v[2], v[3]} {}

    _SPL_CONSTEXPR mat4(vec4 c0, vec4 c1, vec4 c2, vec4 c3)
        : column{c0, c1, c2, c3} {}

    _SPL_CONSTEXPR mat4(const mat4f& m) : column{m[0], m[1], m[2], m[3]} {}

    _SPL_CONSTEXPR vec4& operator [](int i) { return column[i]; }

    _SPL_CONSTEXPR const vec4& operator [](int i) const { return column[i]; }

    _SPL_CONSTEXPR float& operator ()(int i, int j) { return column[j][i]; }

    _SPL_CONSTEXPR float operator ()(int i, int j) const { return column[j][i]; }

    _SPL_CONSTEXPR vec4 operator *(const vec4& v) const
    {
        return v.x * column[0] + v.y * column[1] + v.z * column[2] + v.w * column[3];
    }

    _SPL_CONSTEXPR mat4 operator *(const mat4& m) const
    {
        return {(*this) * m[0], (*this) * m[1], (*this) * m[2], (*this) * m[3]};
    }

    _SPL_CONSTEXPR operator mat3() const
    {
        return {vec3(column[0]), vec3(column[1]), vec3(column[2])};
    }

    _SPL_CONSTEXPR vec4 transform(const vec4& v) const
    {
        return (*this) * v;
    }

    _SPL_CONSTEXPR vec4 transpose_transform(const vec4& v) const
    {
        return { column[0].dot(v), column[1].dot(v), column[2].dot(v), column[3].dot(v) };
    }

    _SPL_CONSTEXPR mat4 transposed() const
    {
        auto& [c0, c1, c2, c3] = column;
        return {{c0.x, c1.x, c2.x, c3.x}, {c0.y, c1.y, c2.y, c3.y},
            {c0.z, c1.z, c2.z, c3.z}, {c0.w, c1.w, c2.w, c3.w}};
    }
};

_SPL_CONSTEXPR mat3 rotation(const vec3& v, float angle)
{
    const auto c = cosf(angle);
    const auto s = sinf(angle);
    const auto c1 = 1.0f - c;
    const auto xx = v.x * v.x;
    const auto yy = v.y * v.y;
    const auto zz = v.z * v.z;
    const auto xy = v.x * v.y;
    const auto xz = v.x * v.z;
    const auto yz = v.y * v.z;
    const auto sx = s * v.x;
    const auto sy = s * v.y;
    const auto sz = s * v.z;
    return mat3
    { // columns
        {c1 * xx + c,  c1 * xy + sz, c1 * xz - sy},
        {c1 * xy - sz, c1 * yy + c,  c1 * yz + sx},
        {c1 * xz + sy, c1 * yz - sx, c1 * zz + c}
    };
}

} // namespace cg::spline
