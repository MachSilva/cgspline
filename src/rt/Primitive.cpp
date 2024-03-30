#include "Primitive.h"

#include <functional>
#include "../Spline.h"

namespace cg::rt
{

Bounds3f Sphere::bounds() const
{
    return {};
}

Bounds3f Sphere::bounds(const mat4f& M) const
{
    return {};
}

// __host__ __device__
bool Sphere::intersect(Intersection& hit, const Ray& ray) const
{
    return {};
}

bool Sphere::intersects(const Ray& ray0) const
{
    return {};
}

// __host__ __device__
vec3f Sphere::normal(const Intersection& hit) const
{
    return {};
}

Bounds3f Mesh::bounds() const
{
    return {};
}

Bounds3f Mesh::bounds(const mat4f& M) const
{
    return {};
}

// __host__ __device__
bool Mesh::intersect(Intersection& hit, const Ray& ray) const
{
    return {};
}

bool Mesh::intersects(const Ray& ray0) const
{
    return {};
}

// __host__ __device__
vec3f Mesh::normal(const Intersection& hit) const
{
    return {};
}

Bounds3f BezierSurface::bounds() const
{
    return spline::boundingbox(std::views::transform(
        std::views::counted(this->indices, this->indexCount),
        [&](uint32_t i) -> vec4f { return this->vertices[i]; }
    ));
}

Bounds3f BezierSurface::bounds(const mat4f& M) const
{
    return spline::boundingbox(std::views::transform(
        std::views::counted(this->indices, this->indexCount),
        [&](uint32_t i) -> vec4f { return M * this->vertices[i]; }
    ));
}

// __host__ __device__
bool BezierSurface::intersect(Intersection& hit0, const Ray& ray0) const
{
    auto fn = [this](Intersection& hit, const Ray& ray, uint32_t index)
    {
        cg::Intersection anotherHit;
        anotherHit.distance = hit.t;
        anotherHit.p = hit.coordinates;
        cg::Ray3f anotherRay { ray.origin, ray.direction };
        anotherRay.tMin = ray.tMin;
        anotherRay.tMax = ray.tMax;
        const uint32_t* patch = this->indices + 16*index;
        if (spline::doBezierClipping(anotherHit, anotherRay, this->vertices, patch))
        {
            hit.index = index;
            hit.object = this;
            hit.t = anotherHit.distance;
            hit.coordinates = anotherHit.p;
            return true;
        }
        return false;
    };
    return bvh->intersect(hit0, ray0, fn);
}

bool BezierSurface::intersects(const Ray& ray0) const
{
    auto fn = [this](const Ray& ray, uint32_t index)
    {
        cg::Intersection anotherHit { .distance = ray.tMax };
        cg::Ray3f anotherRay { ray.origin, ray.direction };
        anotherRay.tMin = ray.tMin;
        anotherRay.tMax = ray.tMax;
        const uint32_t* patch = this->indices + 16*index;
        return spline::doBezierClipping(
            anotherHit, anotherRay, this->vertices, patch);
    };
    return bvh->intersects(ray0, fn);
}

// __host__ __device__
vec3f BezierSurface::normal(const Intersection& hit) const
{
    auto [u, v, _] = hit.coordinates;
    spline::PatchRef patch (this->vertices, this->indices, uint32_t(hit.index));
    return spline::normal(patch, u, v).versor();
}

void BezierSurface::buildBVH(BVH& bvh)
{
    Buffer<BVH::ElementData> patchData (this->patchCount());
    for (int i = 0; i < patchData.size(); i++)
    {
        auto& e = patchData[i];
        const uint32_t* patch = this->indices + 16*i;
        e.bounds = {};
        e.centroid = vec3f{0};
        e.index = i;
        constexpr float inv = 1.0f / 16.0f;
        for (int j = 0; j < 16; j++)
        {
            vec4f& wp = this->vertices[patch[j]];
            vec3f p = spline::project(wp);
            e.bounds.inflate(p);
            e.centroid += inv * p;
        }
    }
    bvh.build(patchData, 1);
    this->bvh = &bvh;
}

} // namespace cg::rt
