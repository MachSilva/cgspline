#pragma once

#include <geometry/BVH.h>
#include <span>
#include "GLBezierSurface.h"

namespace cg
{

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

