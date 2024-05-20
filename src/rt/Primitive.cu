#include "Primitive.h"

#include <geometry/Triangle.h>
#include <functional>
#include "../Spline.h"
#include "../SplineMat.h"

namespace cg::rt
{

// Sphere

HOST DEVICE
Bounds3f Sphere::bounds() const
{
    return {};
}

HOST DEVICE
Bounds3f Sphere::bounds(const mat4& M) const
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
vec3 Sphere::normal(const Intersection& hit) const
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

HOST DEVICE vec3 normal(const Sphere&, const Intersection&)
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
Bounds3f Mesh::bounds(const mat4& M) const
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
vec3 Mesh::normal(const Intersection& hit) const
{
    return rt::normal(*this, hit);
}

void Mesh::buildBVH(BVH& bvh)
{
    Buffer<BVH::ElementData> data (this->indexCount / 3);
    for (int i = 0; i < data.size(); i++)
    {
        auto& e = data[i];
        const uint32_t* v = this->indices + 3*i;
        const vec3 p0 = this->vertices[v[0]];
        const vec3 p1 = this->vertices[v[1]];
        const vec3 p2 = this->vertices[v[2]];
        e.index = i;
        e.centroid = (1.0f / 3.0f) * (p0 + p1 + p2);
        e.bounds = {};
        e.bounds.inflate(p0);
        e.bounds.inflate(p1);
        e.bounds.inflate(p2);
    }
    bvh.build(data, 16);
    this->bvh = &bvh;
}

HOST DEVICE bool intersect(const Mesh& m, Intersection& hit0, const Ray& ray0)
{
    auto fn = [&m](Intersection& hit, const Ray& ray, uint32_t index)
    {
        cg::Ray3f anotherRay { ray.origin, ray.direction };
        anotherRay.tMin = ray.tMin;
        anotherRay.tMax = fmin(ray.tMax, hit.t);
        const uint32_t* v = m.indices + 3*index;
        const auto& p0 = m.vertices[v[0]];
        const auto& p1 = m.vertices[v[1]];
        const auto& p2 = m.vertices[v[2]];
        vec3 p;
        float t;
        if (triangle::intersect(anotherRay, p0, p1, p2, p, t))
        {
            hit.index = index;
            hit.object = &m;
            hit.t = t;
            hit.coordinates = p;
            return true;
        }
        return false;
    };
    return m.bvh->hashIntersect(hit0, ray0, fn);
}

HOST DEVICE bool intersect(const Mesh& m, const Ray& ray0)
{
    auto fn = [&m](const Ray& ray, uint32_t index)
    {
        cg::Ray3f anotherRay { ray.origin, ray.direction };
        anotherRay.tMin = ray.tMin;
        anotherRay.tMax = ray.tMax;
        const uint32_t* v = m.indices + 3*index;
        const auto& p0 = m.vertices[v[0]];
        const auto& p1 = m.vertices[v[1]];
        const auto& p2 = m.vertices[v[2]];
        vec3 p;
        float t;
        return triangle::intersect(anotherRay, p0, p1, p2, p, t);
    };
    return m.bvh->hashIntersect(ray0, fn);
}

HOST DEVICE vec3 normal(const Mesh& m, const Intersection& hit)
{
    const uint32_t* v = m.indices + 3*hit.index;
    return triangle::normal(m.vertices[v[0]], m.vertices[v[1]], m.vertices[v[2]]);
}

// BezierSurface

HOST DEVICE
Bounds3f BezierSurface::bounds() const
{
    return spline::boundingbox<float>(
        this->indices, this->indices + this->indexCount,
        [&](uint32_t i) -> vec4 { return this->vertices[i]; }
    );
}

HOST DEVICE
Bounds3f BezierSurface::bounds(const mat4& M) const
{
    return spline::boundingbox<float>(
        this->indices, this->indices + this->indexCount,
        [&](uint32_t i) -> vec4 { return M * this->vertices[i]; }
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
vec3 BezierSurface::normal(const Intersection& hit) const
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
        e.centroid = vec3{0};
        e.index = i;
        constexpr float inv = 1.0f / 16.0f;
        for (int j = 0; j < 16; j++)
        {
            vec4& wp = this->vertices[patch[j]];
            vec3 p = spline::project(wp);
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

HOST DEVICE vec3 normal(const BezierSurface& s, const Intersection& hit)
{
    auto [u, v, _] = hit.coordinates;
    spline::PatchRef patch (s.vertices, s.indices, uint32_t(hit.index));
    return spline::mat::normal(patch, u, v).versor();
}

} // namespace cg::rt
