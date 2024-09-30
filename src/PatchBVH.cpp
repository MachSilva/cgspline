#include "PatchBVH.h"
#include "Spline.h"

namespace cg
{

PatchBVH::PatchBVH(const Surface* s)
    : BVHBase(1), _surface{s}
{
    init();
}

PatchBVH::PatchBVH(const GLSurface* s) : BVHBase(1)
{
    PatchTable t;

    {
        auto p0 = s->indices()->scopedMap();
        auto p1 = s->points()->scopedMap();

        t.indices.assign(p0.get(), p0.get() + s->indices()->size());
        t.points.assign(p1.get(), p1.get() + s->points()->size());

        t.groups = s->groups();
    }

    if (auto m = s->matrices())
    {
        auto p2 = m->scopedMap();
        t.matrices.assign(p2.get(), p2.get() + m->size());
    }

    _surface = new Surface(std::move(t));

    init();
}

void PatchBVH::init()
{
    auto count = _surface->count();

    auto points = _surface->points().data();
    auto indices = _surface->indices().data();

    PrimitiveInfoArray info (count);
    _primitiveIds.resize(count);

    uint32_t i = 0;
    _surface->patches().forEachPatch([&](PatchData p)
    {
        _primitiveIds[i] = i;
        if (p.type == PatchType_Bezier)
        {
            info[i] = PrimitiveInfo(i,
                spline::boundingbox(points, indices + p.offset)
            );
        }
        i++;
    });

    build(info);
}

bool PatchBVH::intersectLeaf(uint32_t first, uint32_t count, const Ray3f& ray) const
{
    assert(count <= 1); // Max of 1 primitive per node

    auto points = _surface->points().data();
    auto indices = _surface->indices().data();

    Intersection hit;
    hit.distance = math::Limits<float>::inf();

    auto& i = _primitiveIds[first];
    auto d = _surface->patches().find(i);
    const uint32_t* patch = indices + d.offset;
    // return spline::doSubdivision(hit, ray, points, patch);
    return spline::doBezierClipping(hit, ray, points, patch);
}

void PatchBVH::intersectLeaf(uint32_t first,
    uint32_t count,
    const Ray3f& ray,
    Intersection& hit) const
{
    assert(count <= 1); // Max of 1 primitive per node

    auto points = _surface->points().data();
    auto indices = _surface->indices().data();

    uint32_t i = _primitiveIds[first];

    auto d = _surface->patches().find(i);
    const uint32_t* patch = indices + d.offset;
    // if (spline::doSubdivision(hit, ray, points, patch))
    if (spline::doBezierClipping(hit, ray, points, patch))
    {
        hit.triangleIndex = i;
        hit.object = this; // intersectLeaf must set hit.object
    }
}

} // namespace cg
