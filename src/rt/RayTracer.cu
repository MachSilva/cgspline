#include "RayTracer.h"

#include <graphics/Color.h>
#include <geometry/Bounds3.h>
#include <cmath>
#include <utility>
#include <math_constants.h>
#include "PBR.h"

namespace cg::rt
{

struct __align__(8) RayTracer::Context
{
    vec2f topLeftCorner;
    float half_dx;
    float half_dy;
    mat4f cameraToWorld;
    mat3f cameraNormalToWorld;
    vec3f cameraPosition;
    __align__(8) Raw<Frame> frame;
    __align__(8) Raw<const Scene> scene;
    __align__(8) Options options;

    __device__ vec3f trace(const Ray&,
        vec3f attenuation, int depth) const;
    __device__ int intersect(Intersection&, const Ray&) const;
    __device__ bool intersect(const Ray&) const;
    __device__ vec3f miss() const;
    __device__ vec3f anyHit() const;
    __device__ vec3f closestHit(const Intersection&, const Ray&,
        vec3f attenuation, uint32_t object, int depth) const;

    __device__
    vec3f pixelRayDirection(int x, int y) const
    {
        vec3f P
        {
            this->topLeftCorner.x + this->half_dx * (2*x),
            this->topLeftCorner.y - this->half_dy * (2*y),
            -1.0f
        };
        P = this->cameraToWorld.transform3x4(P);
        return (P - this->cameraPosition).versor();
    }
};

using Context = RayTracer::Context;
using Key = Scene::Key;

__global__ void render(Context*);

void RayTracer::render(Frame* frame, const Camera* camera, const Scene* scene,
    cudaStream_t stream)
{
    CUDA_CHECK(cudaMemPrefetchAsync(frame->data(), frame->size_bytes(),
        _options.device, stream));

    auto& objs = scene->objects;
    std::apply(
        [device=_options.device,n=objs.size(),stream]
        (auto*... ptrs) -> void
        {
            ((CUDA_CHECK(cudaMemPrefetchAsync(ptrs,
                n * sizeof (decltype(*ptrs)), device, stream))), ...);
        },
        objs.as_pointer_tuple()
    );

    {
        Context* ptr;
        CUDA_CHECK(cudaMalloc(&ptr, sizeof (Context)));
        _ctx = {ptr, [](void* p){ cudaFree(p); }};
    }

    uint32_t w = frame->width();
    uint32_t h = frame->height();

    {
        Context ctx;
        ctx.options = _options;
        ctx.frame = *frame;
        ctx.scene = *scene;

        // Camera data
        ctx.cameraPosition = camera->transform.position;
        ctx.cameraToWorld = camera->transform.matrix();
        ctx.cameraNormalToWorld = camera->transform.normalMatrix();

        auto fovy = camera->fieldOfView;
        auto halfHeight = tanf(0.5f * fovy);
        ctx.half_dy = halfHeight / h;
        ctx.half_dx = ctx.half_dy; // * camera->aspectRatio;
        ctx.topLeftCorner.x = -ctx.half_dx * (w - 1);
        ctx.topLeftCorner.y = halfHeight - ctx.half_dy;

        if (_options.flipYAxis)
        {
            ctx.topLeftCorner.y = -ctx.topLeftCorner.y;
            ctx.half_dy = -ctx.half_dy;
        }
        // CUDA_CHECK();
        cudaMemcpyAsync(_ctx.get(), &ctx, sizeof (ctx), cudaMemcpyHostToDevice,
            stream);
    }

    dim3 threadsPerBlock (8, 8);
    dim3 blocksPerGrid
    {
        (w / 8) + (w % 8 ? 1 : 0),
        (h / 8) + (h % 8 ? 1 : 0)
    };

    rt::render<<<blocksPerGrid, threadsPerBlock, 0, stream>>>(_ctx.get());

    CUDA_CHECK(cudaDeviceSynchronize());
}

__global__
void render(Context* ctx)
{
    auto i = blockIdx.x * blockDim.x + threadIdx.x;
    auto j = blockIdx.y * blockDim.y + threadIdx.y;
    if (i >= ctx->frame->width() || j >= ctx->frame->height())
        return;

    auto d = ctx->pixelRayDirection(i, j);
    Ray pixelRay
    {
        .origin = ctx->cameraPosition,
        .direction = d,
        .tMin = 0.001f,
        .tMax = numeric_limits<float>::infinity()
    };
    vec3f c = ctx->trace(pixelRay, vec3f(1), ctx->options.recursionDepth);
    c.x = math::clamp(c.x, 0.0f, 1.0f);
    c.y = math::clamp(c.y, 0.0f, 1.0f);
    c.z = math::clamp(c.z, 0.0f, 1.0f);
    ctx->frame->at(i, j) = pack_sRGB(c.x, c.y, c.z);
}

__device__
vec3f Context::trace(const Ray& ray, vec3f attenuation, int depth) const
{
    Intersection hit
    {
        .object = nullptr,
        .t = ray.tMax,
    };
    int nearestObject = intersect(hit, ray);

    if (nearestObject < 0)
        return miss();

    // Is opaque?
    // TODO refraction

    return closestHit(hit, ray, attenuation, nearestObject, depth);
}

__device__
int Context::intersect(Intersection& hit0, const Ray& ray0) const
{
    int nearestObject = -1;
    auto fn = [&](Intersection& hit, const Ray& ray, uint32_t i)
    {
        Intersection localHit;
        auto& objs = this->scene->objects;
        auto p = objs.get<Key::ePrimitive>(i);
        auto& M_1 = objs.get<Key::eWorld2LocalMatrix>(i);
        Ray localRay
        {
            .origin = M_1.transform(ray.origin),
            .direction = M_1.transformVector(ray.direction),
        };
        auto d = localRay.direction.length();
        localRay.tMin = ray.tMin * d;
        localRay.tMax = ray.tMax * d;
        d = 1 / d;
        localRay.direction *= d;
        localHit.t = localRay.tMax;

        bool b = false;
        switch (objs.get<Key::ePrimitiveType>(i))
        {
        case PrimitiveType::eBezierSurface:
            b = rt::intersect(*(const BezierSurface*)p, localHit, localRay);
            break;
        case PrimitiveType::eMesh:
            b = rt::intersect(*(const Mesh*)p, localHit, localRay);
            break;
        default:
            return false;
        }

        if (b)
        {
            auto t = localHit.t * d;
            if (t < hit.t)
            {
                hit.coordinates = localHit.coordinates;
                hit.index = localHit.index;
                hit.object = &p;
                hit.t = t;
                nearestObject = i;
                return true;
            }
        }
        return false;
    };
    this->scene->topLevelBVH.hashIntersect(hit0, ray0, fn);
    return nearestObject;
}

__device__
bool Context::intersect(const Ray& ray0) const
{
    auto fn = [this](const Ray& ray, uint32_t i)
    {
        auto& objs = this->scene->objects;
        auto p = objs.get<Key::ePrimitive>(i);
        auto& M_1 = objs.get<Key::eWorld2LocalMatrix>(i);
        Ray localRay
        {
            .origin = M_1.transform(ray.origin),
            .direction = M_1.transformVector(ray.direction),
        };
        auto d = localRay.direction.length();
        localRay.tMin = ray.tMin * d;
        localRay.tMax = ray.tMax * d;
        d = 1 / d;
        localRay.direction *= d;

        switch (objs.get<Key::ePrimitiveType>(i))
        {
        case PrimitiveType::eBezierSurface:
            return rt::intersect(*(const BezierSurface*)p, localRay);
        case PrimitiveType::eMesh:
            return rt::intersect(*(const Mesh*)p, localRay);
        default:
            return false;
        }

        // return p->intersect(localRay);
    };
    return this->scene->topLevelBVH.hashIntersect(ray0, fn);
}

__device__
vec3f Context::miss() const
{
    return this->options.backgroundColor;
}

__device__
vec3f Context::anyHit() const
{
    return {};
}

__device__
vec3f Context::closestHit(const Intersection& hit, const Ray& ray,
    vec3f attenuation, uint32_t object, int depth) const
{
    const auto p = this->scene->objects.get<Key::ePrimitive>(object);
    const auto& m = this->scene->objects.get<Key::eMaterial>(object);
    const auto& M_1 = this->scene->objects.get<Key::eWorld2LocalMatrix>(object);
    vec3f color {0};

    // m.specular.xyz = max(m.specular.xyz, vec3f(0.04));

    // From the point to the camera; BRDF uses this vector orientation.
    vec3f V = - ray.direction;
    // vec3f N = p->normal(hit);
    // vec3f N = (mat3f(M_1).transposed() * p->normal(hit)).versor();

    vec3f N;
    switch (this->scene->objects.get<Key::ePrimitiveType>(object))
    {
    case PrimitiveType::eBezierSurface:
        N = normal(*(const BezierSurface*)p, hit);
        break;
    case PrimitiveType::eMesh:
        N = normal(*(const Mesh*)p, hit);
        break;
    default:
        return color;
    }
    N = (mat3f(M_1).transposed() * N).versor();

    // bool backfaced = false;
    float dotNV = vec3f::dot(N, V);

    if (dotNV < 0)
    {
        // backfaced = true;
        N = -N;
        dotNV = -dotNV;
    }

    vec3f P = ray.origin + hit.t * ray.direction;

    const auto& lights = this->scene->lights;
    for (int i = 0; i < lights.size(); i++)
    {
        vec3f I;
        vec3f L;
        float d;

        if (!lightVector(d, L, P, lights[i]))
            continue;

        const float dotNL = vec3f::dot(N, L);
        if (dotNL < 1e-14f) // Avoid division by zero and backfaced lights
            continue;

        // Shadow
        Ray r1 { .origin = P + this->options.eps * L, .direction = L, .tMax = d };
        if (intersect(r1))
            continue;

        I = lightColor(d, lights[i]);
        color += BRDF(I, L, V, N, dotNV, dotNL, m);
    }

    color *= std::numbers::pi_v<float> * attenuation;

    if (depth <= 0)
        return color;

    // Reflection
    // constexpr float minRadiance = 0x1p-8f;
    // vec3f reflectance = schlick(m.specular, dotNV);
    // vec3f a = attenuation * reflectance;
    // if (a.max() > minRadiance)
    // {
    //     // I = -V
    //     vec3f R = (-V) + 2 * dotNV * N;
    //     Ray r
    //     {
    //         .origin = P + this->options.eps * R,
    //         .direction = R,
    //         .tMax = numeric_limits<float>::infinity()
    //     };
    //     // color += m.opacity * trace(r, attenuation, depth - 1);
    //     color += trace(r, a, depth - 1);
    // }

    return color;
}

} // namespace cg::rt
