#pragma once

#include <cinttypes>
#include <math/Vector4.h>
#include "BVH.h"
#include "CUDAUtility.h"

namespace cg::rt
{

struct Primitive
{
    virtual Bounds3f bounds() const = 0;

    virtual Bounds3f bounds(const mat4f& transform) const = 0;

    // __host__ __device__
    virtual bool intersect(Intersection& hit, const Ray& ray) const = 0;

    virtual bool intersect(const Ray& ray) const = 0;

    // __host__ __device__
    virtual vec3f normal(const Intersection& hit) const = 0;
};

struct Sphere : public Primitive
{
    Bounds3f bounds() const override;

    Bounds3f bounds(const mat4f& transform) const override;

    // __host__ __device__
    bool intersect(Intersection& hit, const Ray& ray) const override;

    bool intersect(const Ray& ray) const override;

    // __host__ __device__
    vec3f normal(const Intersection& hit) const override;

    vec4f position;
    float radius;
};

struct Mesh : public Primitive
{
    Bounds3f bounds() const override;

    Bounds3f bounds(const mat4f& transform) const override;

    // __host__ __device__
    bool intersect(Intersection& hit, const Ray& ray) const override;

    bool intersect(const Ray& ray) const override;

    // __host__ __device__
    vec3f normal(const Intersection& hit) const override;

    /**
     * Mesh is formed by triangles.
     * Each triangle is a set of 3 vertices.
     */
    const BVH* bvh;
    vec3f* vertices;
    vec3f* normals;
    uint32_t* indices; // Index of the first triangle at index array.
    uint32_t indexCount;
};

/**
 * BezierSurface is formed by bicubic Bézier patches.
 * Each patch is a set of 16 vertices.
 */
struct BezierSurface : public Primitive
{
    Bounds3f bounds() const override;

    Bounds3f bounds(const mat4f& transform) const override;

    // __host__ __device__
    bool intersect(Intersection& hit, const Ray& ray) const override;

    bool intersect(const Ray& ray) const override;

    // __host__ __device__
    vec3f normal(const Intersection& hit) const override;

    // __host__
    void buildBVH(BVH& bvh);

    auto patchCount() const noexcept { return this->indexCount / 16; }

    const BVH* bvh;
    vec4f* vertices;
    uint32_t* indices; // Index of the first patch at index array.
    uint32_t indexCount;
};

constexpr size_t __[]
{
    sizeof (Primitive),
    sizeof (Sphere),
    sizeof (Mesh),
    sizeof (BezierSurface),
};

} // namespace cg::rt
