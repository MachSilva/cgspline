#include "RayTracer.h"

#include <graphics/Color.h>
#include <geometry/Bounds3.h>
#include <cmath>
#include <utility>
#include <math_constants.h>
#include <cuda_gl_interop.h>
#include "PBR.h"

namespace cg::rt
{

__managed__ Frame* g_BezierClippingHeatMap;

struct RayPayload
{
    Ray ray;
    vec3 color;
    vec3 attenuation;
    int depth;
};

struct RayTracer::Context
{
    __align__(16) Raw<Frame> frame;
    __align__(16) Raw<const Scene> scene;
    __align__(16) Options options;
    __align__(16) mat4 cameraToWorld;
    __align__(16) mat3 cameraNormalToWorld;
    __align__(16) vec3 cameraPosition;
    vec2 topLeftCorner;
    float half_dx;
    float half_dy;

    __device__ bool anyHit(RayPayload&) const;
    __device__ bool closestHit(const Intersection&, RayPayload&, uint32_t object) const;
    __device__ bool intersect(const Ray&) const;
    __device__ int  intersect(Intersection&, const Ray&) const;
    __device__ bool miss(RayPayload&) const;
    __device__ bool trace(RayPayload&) const;

    __device__
    vec3 pixelRayDirection(int x, int y) const
    {
        vec4 P
        {
            this->topLeftCorner.x + this->half_dx * (2*x),
            this->topLeftCorner.y - this->half_dy * (2*y),
            -1.0f,
            1.0f
        };
        P = this->cameraToWorld.transform(P);
        return (project(P) - this->cameraPosition).versor();
    }
};

using Context = RayTracer::Context;
using Key = Scene::Key;

__global__ void render(Context*);

RayTracer::RayTracer()
{
    CUDA_CHECK(cudaMalloc(&_ctx, sizeof (Context)));
    CUDA_CHECK(cudaFuncSetCacheConfig(&rt::render, cudaFuncCachePreferL1));
}

RayTracer::~RayTracer()
{
    cudaFree(_ctx);
}

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

    uint32_t w = frame->width();
    uint32_t h = frame->height();

    g_BezierClippingHeatMap = _options.heatMap;

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
        CUDA_CHECK(cudaMemcpyAsync(_ctx, &ctx, sizeof (ctx),
            cudaMemcpyHostToDevice, stream));
    }

    constexpr auto N = 16;
    dim3 threadsPerBlock (N, N);
    dim3 blocksPerGrid
    {
        (w / N) + (w % N ? 1 : 0),
        (h / N) + (h % N ? 1 : 0)
    };

    rt::render<<<blocksPerGrid, threadsPerBlock, 0, stream>>>(_ctx);
}

__global__ //__launch_bounds__(64)
void render(Context* ctx)
{
    auto i = blockIdx.x * blockDim.x + threadIdx.x;
    auto j = blockIdx.y * blockDim.y + threadIdx.y;
    if (i >= ctx->frame->width() || j >= ctx->frame->height())
        return;

    auto d = ctx->pixelRayDirection(i, j);
    RayPayload payload
    {
        .ray = {
            .origin = ctx->cameraPosition,
            .direction = d,
            .tMin = 0.001f,
            .tMax = numeric_limits<float>::infinity()
        },
        .color = vec3f(0),
        .attenuation = vec3f(1),
        .depth = ctx->options.recursionDepth
    };
    while (ctx->trace(payload));
    vec3f c = payload.color;
    c.x = __saturatef(c.x);
    c.y = __saturatef(c.y);
    c.z = __saturatef(c.z);
    ctx->frame->at(i, j) = pack_sRGB(c.x, c.y, c.z);
}

__device__
bool Context::trace(RayPayload& payload) const
{
    Intersection hit
    {
        .object = nullptr,
        .t = payload.ray.tMax,
    };
    int nearestObject = intersect(hit, payload.ray);

    if (nearestObject < 0)
        return miss(payload);

    // Is opaque?
    // TODO refraction

    return closestHit(hit, payload, nearestObject);
}

__device__
int Context::intersect(Intersection& hit0, const Ray& ray0) const
{
    int nearestObject = -1;
    auto fn = [&](Intersection& hit, const Ray& ray, uint32_t i) -> bool
    {
        Intersection localHit;
        auto& objs = this->scene->objects;
        auto p = objs.get<Key::ePrimitive>(i);
        auto& M_1 = objs.get<Key::eWorld2LocalMatrix>(i);
        Ray localRay
        {
            .origin = project(M_1.transform(vec4(ray.origin, 1))),
            .direction = mat3(M_1).transform(ray.direction),
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
            __builtin_unreachable();
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
    auto fn = [this](const Ray& ray, uint32_t i) -> bool
    {
        auto& objs = this->scene->objects;
        auto p = objs.get<Key::ePrimitive>(i);
        auto& M_1 = objs.get<Key::eWorld2LocalMatrix>(i);
        Ray localRay
        {
            .origin = project(M_1.transform(vec4(ray.origin, 1))),
            .direction = mat3(M_1).transform(ray.direction),
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
            __builtin_unreachable();
        }
    };
    return this->scene->topLevelBVH.hashIntersect(ray0, fn);
}

__device__ bool Context::miss(RayPayload& payload) const
{
    payload.color += this->scene->backgroundColor * payload.attenuation;
    return false;
}

__device__
bool Context::anyHit(RayPayload& payload) const
{
    return false;
}

__device__
bool Context::closestHit(const Intersection& hit, RayPayload& payload, uint32_t object) const
{
    const auto p = this->scene->objects.get<Key::ePrimitive>(object);
    const auto& m = this->scene->objects.get<Key::eMaterial>(object);
    const auto& M_1 = this->scene->objects.get<Key::eWorld2LocalMatrix>(object);
    vec3f color {0};

    // From the point to the camera; BRDF uses this vector orientation.
    vec3f V = - payload.ray.direction;

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
        __builtin_unreachable();
    }
    N = (mat3(M_1).transposed() * N).versor();

    // bool backfaced = false;
    float dotNV = vec3f::dot(N, V);

    if (dotNV < 0)
    {
        // backfaced = true;
        N = -N;
        dotNV = -dotNV;
    }

    vec3f P = payload.ray.origin + hit.t * payload.ray.direction;

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

    payload.color += color * std::numbers::pi_v<float> * payload.attenuation;

    if (payload.depth <= 0)
        return false;

    // Reflection
    constexpr float minRadiance = 0x1p-8f;
    float r = m.roughness;
    vec3f reflectance = (1 - r*r) * schlick(m.specular, dotNV);
    vec3f a = payload.attenuation * reflectance;
    if (a.max() <= minRadiance)
        return false;

    // I = -V
    vec3f R = (-V) + 2 * dotNV * N;
    payload.ray =
    {
        .origin = P + this->options.eps * R,
        .direction = R,
        .tMax = numeric_limits<float>::infinity()
    };
    payload.attenuation = a;
    payload.depth = payload.depth - 1;
    // color += m.opacity * trace(r, attenuation, depth - 1);
    // color += trace(r, a, depth - 1);
    return true;
}

} // namespace cg::rt
