#include "PatchBVH.h"

namespace cg
{

PatchBVH::PatchBVH(const BezierPatches& patches) : BVHBase(1), _patches{&patches}
{
    // build({});
}

bool PatchBVH::intersectLeaf(uint32_t first, uint32_t count, const Ray3f& ray) const
{
    return false;
}

void PatchBVH::intersectLeaf(uint32_t first,
    uint32_t count,
    const Ray3f& ray,
    Intersection& hit) const
{
    0;
}


} // namespace cg

