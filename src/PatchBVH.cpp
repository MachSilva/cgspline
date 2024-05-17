#include "PatchBVH.h"
#include "Spline.h"

namespace cg
{

PatchBVH::PatchBVH(const BezierSurface* patches)
    : BVHBase(1), _patches{patches}
{
    init();
}

PatchBVH::PatchBVH(const GLBezierSurface* patches) : BVHBase(1)
{
    auto p0 = patches->indices()->map();
    auto p1 = patches->points()->map();

    std::vector<uint32_t> indices
        (p0.get(), p0.get() + patches->indices()->size());
    std::vector<vec4> points
        (p1.get(), p1.get() + patches->points()->size());

    _patches = new BezierSurface(std::move(points), std::move(indices));

    init();
}

void PatchBVH::init()
{
    auto count = _patches->count();

    auto points = _patches->points().data();
    auto indices = _patches->indices().data();

    PrimitiveInfoArray info (count);
    _primitiveIds.resize(count);

    for (uint32_t i = 0; i < count; i++)
    {
        _primitiveIds[i] = i;
        info[i] = PrimitiveInfo(i,
            spline::boundingbox(points, indices + 16*i)
        );
    }

    build(info);
}

bool PatchBVH::intersectLeaf(uint32_t first, uint32_t count, const Ray3f& ray) const
{
    assert(count == 1); // Max of 1 primitive per node

    auto points = _patches->points().data();
    auto indices = _patches->indices().data();

    Intersection hit;
    hit.distance = math::Limits<float>::inf();

    auto& i = _primitiveIds[first];
    const uint32_t* patch = indices + 16*i;
    // return spline::doSubdivision(hit, ray, points, patch);
    return spline::doBezierClipping(hit, ray, points, patch);
}

void PatchBVH::intersectLeaf(uint32_t first,
    uint32_t count,
    const Ray3f& ray,
    Intersection& hit) const
{
    assert(count == 1); // Max of 1 primitive per node

    auto points = _patches->points().data();
    auto indices = _patches->indices().data();

    uint32_t i = _primitiveIds[first];

    const uint32_t* patch = indices + 16*i;
    // if (spline::doSubdivision(hit, ray, points, patch))
    if (spline::doBezierClipping(hit, ray, points, patch))
    {
        hit.triangleIndex = i;
        hit.object = this; // intersectLeaf must set hit.object
    }
}

} // namespace cg
