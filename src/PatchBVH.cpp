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
    auto matrices = _surface->patches().matrices.data();

    PrimitiveInfoArray info (count);
    _primitiveIds.resize(count);

    vec4f temp[20];

    uint32_t i = 0;
    _surface->patches().forEachPatch([&](PatchTable::PatchRef p)
    {
        const int n = patchTypeOriginalSize(p.type);
        _primitiveIds[i] = i;
        auto pIndex = indices + p.offset;
        if (p.matrixOffset >= 0)
        {
            auto pMatrix = matrices + p.matrixOffset;
            // multiply matrix
            for (int row = 0; row < n; row++)
            {
                temp[row] = pMatrix[row] * points[pIndex[0]];
                for (int col = 1; col < p.size; col++)
                    temp[row] += pMatrix[row + col*n] * points[pIndex[col]];
            }
            info[i] = PrimitiveInfo(i, spline::boundingbox(temp, temp+n));
        }
        else
        {
            info[i] = PrimitiveInfo(i,
                spline::boundingbox<float>(pIndex, pIndex+n, [&](GLuint idx)
                {
                    return points[idx];
                })
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
