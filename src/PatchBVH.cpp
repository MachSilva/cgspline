#include "PatchBVH.h"
#include "Spline.h"

namespace cg
{

PatchBVH::PatchBVH(const BezierPatches& patches) : BVHBase(1), _patches{&patches}
{
    auto count = _patches->count();
    auto indexes = _patches->indexes()->map(GL_READ_ONLY);
    auto points = _patches->points()->map(GL_READ_ONLY);

    PrimitiveInfoArray info (count);
    _primitiveIds.resize(count);

    for (uint32_t i = 0; i < count; i++)
    {
        _primitiveIds[i] = i;
        info[i] = PrimitiveInfo(i, spline::boundingbox(points.get(), &indexes[16*i]));
    }

    build(info);
}

void PatchBVH::map()
{
    _points = static_cast<decltype(_points)>(
        glMapNamedBuffer(_patches->points()->buffer(), GL_READ_ONLY)
    );
    _indexes = static_cast<decltype(_indexes)>(
        glMapNamedBuffer(_patches->indexes()->buffer(), GL_READ_ONLY)
    );
}

void PatchBVH::unmap()
{
    glUnmapNamedBuffer(_patches->indexes()->buffer());
    glUnmapNamedBuffer(_patches->points()->buffer());
    _indexes = nullptr;
    _points = nullptr;
}

bool PatchBVH::intersectLeaf(uint32_t first, uint32_t count, const Ray3f& ray) const
{
    assert(count == 1); // Max of 1 primitive per node
    // Assert that the buffers are mapped
    assert(_points != nullptr && _indexes != nullptr);

    Intersection hit;
    hit.distance = math::Limits<float>::inf();

    auto& i = _primitiveIds[first];
    const uint32_t* patch = _indexes + 16*i;
    return spline::doSubdivision(hit, ray, _points, patch);
}

void PatchBVH::intersectLeaf(uint32_t first,
    uint32_t count,
    const Ray3f& ray,
    Intersection& hit) const
{
    assert(count == 1); // Max of 1 primitive per node
    // Assert that the buffers are mapped
    assert(_points != nullptr && _indexes != nullptr);

    uint32_t i = _primitiveIds[first];

    const uint32_t* patch = _indexes + 16*i;
    if (spline::doSubdivision(hit, ray, _points, patch))
    {
        hit.triangleIndex = i;
        hit.object = this; // intersectLeaf must set hit.object
    }
}


} // namespace cg

