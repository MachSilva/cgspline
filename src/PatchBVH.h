#pragma once

#include <geometry/BVH.h>
#include <span>
#include "GLSurface.h"

namespace cg
{

/**
 * @brief BVH for surface objects consisting of multiple patches.
 */
class PatchBVH final : public BVHBase
{
public:
    PatchBVH(const Surface*);
    PatchBVH(const GLSurface*);

    Surface* surface() const { return _surface; }

private:
    Ref<Surface> _surface;

    void init();

    bool intersectLeaf(uint32_t, uint32_t, const Ray3f&) const override;
    void intersectLeaf(uint32_t, uint32_t, const Ray3f&, Intersection&) const override;
};

} // namespace cg

