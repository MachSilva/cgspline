#pragma once

#include <geometry/BVH.h>
#include <span>
#include "BezierPatches.h"

namespace cg
{

namespace spline
{

bool bezierClipping(std::span<vec4f> vertices,
    std::span<uint32_t,16> patch,
    const Ray3f&,
    Intersection&);

} // namespace spline

/**
 * @brief BVH for surface objects consisting of multiple patches.
 * @warning You must call @a map() to map opengl buffers to host memory before
 * attempting an intersection. When done, call @a unmap() to release the memory.
 */
class PatchBVH final : public BVHBase
{
public:
    PatchBVH(const BezierPatches&);

    void map();
    void unmap();

protected:
    Reference<BezierPatches> _patches;
    const vec4f* _points {};
    const uint32_t* _indexes {};

    bool intersectLeaf(uint32_t, uint32_t, const Ray3f&) const override;
    void intersectLeaf(uint32_t, uint32_t, const Ray3f&, Intersection&) const override;
};

} // namespace cg

