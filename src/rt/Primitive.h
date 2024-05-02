#pragma once

#include <cinttypes>
#include <math/Vector4.h>
#include "BVH.h"
#include "CUDAUtility.h"

namespace cg::rt
{

enum struct PrimitiveType : uint32_t
{
    eNone = 0,
    eSphere,
    eMesh,
    eBezierSurface
};

struct __align__(8) Primitive
{
    HOST DEVICE
    virtual Bounds3f bounds() const = 0;

    HOST DEVICE
    virtual Bounds3f bounds(const mat4f& transform) const = 0;

    HOST DEVICE
    virtual bool intersect(Intersection& hit, const Ray& ray) const = 0;

    HOST DEVICE
    virtual bool intersect(const Ray& ray) const = 0;

    HOST DEVICE
    virtual vec3f normal(const Intersection& hit) const = 0;
};

struct Sphere final : public Primitive
{
    HOST DEVICE
    Bounds3f bounds() const override;

    HOST DEVICE
    Bounds3f bounds(const mat4f& transform) const override;

    HOST DEVICE
    bool intersect(Intersection& hit, const Ray& ray) const override;

    HOST DEVICE
    bool intersect(const Ray& ray) const override;

    HOST DEVICE
    vec3f normal(const Intersection& hit) const override;

    vec4f position;
    float radius;
};

struct Mesh final : public Primitive
{
    HOST DEVICE
    Bounds3f bounds() const override;

    HOST DEVICE
    Bounds3f bounds(const mat4f& transform) const override;

    HOST DEVICE
    bool intersect(Intersection& hit, const Ray& ray) const override;

    HOST DEVICE
    bool intersect(const Ray& ray) const override;

    HOST DEVICE
    vec3f normal(const Intersection& hit) const override;

    void buildBVH(BVH& bvh);

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
struct BezierSurface final : public Primitive
{
    HOST DEVICE
    Bounds3f bounds() const override;

    HOST DEVICE
    Bounds3f bounds(const mat4f& transform) const override;

    HOST DEVICE
    bool intersect(Intersection& hit, const Ray& ray) const override;

    HOST DEVICE
    bool intersect(const Ray& ray) const override;

    HOST DEVICE
    vec3f normal(const Intersection& hit) const override;

    void buildBVH(BVH& bvh);

    HOST DEVICE
    auto patchCount() const noexcept { return this->indexCount / 16; }

    const BVH* bvh;
    vec4f* vertices;
    uint32_t* indices; // Index of the first patch at index array.
    uint32_t indexCount;
};

// Dynamic (and static) dispatch for host methods
// Static dispatch for device methods

HOST DEVICE bool intersect(const Sphere&, Intersection&, const Ray&);
HOST DEVICE bool intersect(const Mesh&, Intersection&, const Ray&);
HOST DEVICE bool intersect(const BezierSurface&, Intersection&, const Ray&);

HOST DEVICE bool intersect(const Sphere&, const Ray&);
HOST DEVICE bool intersect(const Mesh&, const Ray&);
HOST DEVICE bool intersect(const BezierSurface&, const Ray&);

HOST DEVICE vec3f normal(const Sphere&, const Intersection&);
HOST DEVICE vec3f normal(const Mesh&, const Intersection&);
HOST DEVICE vec3f normal(const BezierSurface&, const Intersection&);

} // namespace cg::rt
