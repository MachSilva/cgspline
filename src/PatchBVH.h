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

class PatchBVH : public BVHBase
{
public:
    PatchBVH(const BezierPatches&);

private:
    Reference<BezierPatches> _patches;

    bool intersectLeaf(uint32_t, uint32_t, const Ray3f&) const override;
    void intersectLeaf(uint32_t, uint32_t, const Ray3f&, Intersection&) const override;
};

} // namespace cg

