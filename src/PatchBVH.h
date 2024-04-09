#pragma once

#include <geometry/BVH.h>
#include <span>
#include "GLBezierSurface.h"

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
 */
class PatchBVH final : public BVHBase
{
public:
    PatchBVH(const BezierSurface*);
    PatchBVH(const GLBezierSurface*);

    BezierSurface* surface() const { return _patches; }

private:
    Ref<BezierSurface> _patches;

    void init();

    bool intersectLeaf(uint32_t, uint32_t, const Ray3f&) const override;
    void intersectLeaf(uint32_t, uint32_t, const Ray3f&, Intersection&) const override;
};

} // namespace cg

