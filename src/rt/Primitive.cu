#include "Primitive.h"

#include <functional>
#include "../Spline.h"

namespace cg::rt
{

// Sphere

HOST DEVICE
Bounds3f Sphere::bounds() const
{
    return {};
}

HOST DEVICE
Bounds3f Sphere::bounds(const mat4f& M) const
{
    return {};
}

HOST DEVICE
bool Sphere::intersect(Intersection& hit, const Ray& ray) const
{
    return rt::intersect(*this, hit, ray);
}

HOST DEVICE
bool Sphere::intersect(const Ray& ray) const
{
    return rt::intersect(*this, ray);
}

HOST DEVICE
vec3f Sphere::normal(const Intersection& hit) const
{
    return rt::normal(*this, hit);
}

HOST DEVICE bool intersect(const Sphere&, Intersection&, const Ray&)
{
    return {};
}

HOST DEVICE bool intersect(const Sphere&, const Ray&)
{
    return {};
}

HOST DEVICE vec3f normal(const Sphere&, const Intersection&)
{
    return {};
}

// Mesh

HOST DEVICE
Bounds3f Mesh::bounds() const
{
    return {};
}

HOST DEVICE
Bounds3f Mesh::bounds(const mat4f& M) const
{
    return {};
}

HOST DEVICE
bool Mesh::intersect(Intersection& hit, const Ray& ray) const
{
    return rt::intersect(*this, hit, ray);
}

HOST DEVICE
bool Mesh::intersect(const Ray& ray) const
{
    return rt::intersect(*this, ray);
}

HOST DEVICE
vec3f Mesh::normal(const Intersection& hit) const
{
    return rt::normal(*this, hit);
}

HOST DEVICE bool intersect(const Mesh&, Intersection&, const Ray&)
{
    return {};
}

HOST DEVICE bool intersect(const Mesh&, const Ray&)
{
    return {};
}

HOST DEVICE vec3f normal(const Mesh&, const Intersection&)
{
    return {};
}

// BezierSurface

HOST DEVICE
Bounds3f BezierSurface::bounds() const
{
    return spline::boundingbox<float>(
        this->indices, this->indices + this->indexCount,
        [&](uint32_t i) -> vec4f { return this->vertices[i]; }
    );
}

HOST DEVICE
Bounds3f BezierSurface::bounds(const mat4f& M) const
{
    return spline::boundingbox<float>(
        this->indices, this->indices + this->indexCount,
        [&](uint32_t i) -> vec4f { return M * this->vertices[i]; }
    );
}

HOST DEVICE
bool BezierSurface::intersect(Intersection& hit, const Ray& ray) const
{
    return rt::intersect(*this, hit, ray);
}

HOST DEVICE
bool BezierSurface::intersect(const Ray& ray) const
{
    return rt::intersect(*this, ray);
}

HOST DEVICE
vec3f BezierSurface::normal(const Intersection& hit) const
{
    return rt::normal(*this, hit);
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

HOST DEVICE bool intersect(const BezierSurface& s, Intersection& hit0, const Ray& ray0)
{
    auto fn = [&s](Intersection& hit, const Ray& ray, uint32_t index)
    {
        cg::Intersection anotherHit;
        anotherHit.distance = hit.t;
        anotherHit.p = hit.coordinates;
        cg::Ray3f anotherRay { ray.origin, ray.direction };
        anotherRay.tMin = ray.tMin;
        anotherRay.tMax = ray.tMax;
        const uint32_t* patch = s.indices + 16*index;
        if (spline::doBezierClipping(anotherHit, anotherRay, s.vertices, patch))
        {
            hit.index = index;
            hit.object = &s;
            hit.t = anotherHit.distance;
            hit.coordinates = anotherHit.p;
            return true;
        }
        return false;
    };
    return s.bvh->hashIntersect(hit0, ray0, fn);
}

HOST DEVICE bool intersect(const BezierSurface& s, const Ray& ray0)
{
    auto fn = [&s](const Ray& ray, uint32_t index)
    {
        cg::Intersection anotherHit { .distance = ray.tMax };
        cg::Ray3f anotherRay { ray.origin, ray.direction };
        anotherRay.tMin = ray.tMin;
        anotherRay.tMax = ray.tMax;
        const uint32_t* patch = s.indices + 16*index;
        return spline::doBezierClipping(
            anotherHit, anotherRay, s.vertices, patch);
    };
    return s.bvh->hashIntersect(ray0, fn);
}

HOST DEVICE vec3f normal(const BezierSurface& s, const Intersection& hit)
{
    auto [u, v, _] = hit.coordinates;
    spline::PatchRef patch (s.vertices, s.indices, uint32_t(hit.index));
    return spline::normal(patch, u, v).versor();
}

} // namespace cg::rt
