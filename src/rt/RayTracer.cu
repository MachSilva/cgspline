#include "RayTracer.h"

#include <graphics/Color.h>
#include <geometry/Bounds3.h>
#include <cmath>
#include <utility>
#include <math_constants.h>

namespace cg::rt
{

using Context = RayTracer::Context;

__global__ void generateHashes(BVH*);
__global__ void render(Context*);
__device__ vec3f trace(const Context*, const Ray&,
    vec3f attenuation, int depth);
__device__ int intersect(const Context*, Intersection&, const Ray&);
__device__ vec3f miss(const Context*);
__device__ vec3f anyHit(const Context*);
__device__ vec3f closestHit(const Context*, const Intersection&, const Ray&,
    vec3f attenuation, uint32_t object, int depth);

void RayTracer::render(Frame* frame, const Camera* camera, const Scene* scene)
{
    CUDA_CHECK(cudaMemPrefetchAsync(frame->data(), frame->size_bytes(),
        _options.device));

    auto& objs = scene->objects;
    std::apply(
        [&](auto*... ptrs) -> void
        {
            ((CUDA_CHECK(cudaMemPrefetchAsync(ptrs,
                sizeof (decltype(*ptrs)), _options.device))), ...);
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
        cudaMemcpy(_ctx.get(), &ctx, sizeof (ctx), cudaMemcpyHostToDevice);
    }

    dim3 threadsPerBlock (32, 32);
    dim3 blocksPerGrid
    {
        (w / 32) + (w % 32 ? 1 : 0),
        (h / 32) + (h % 32 ? 1 : 0)
    };

    rt::render<<<blocksPerGrid, threadsPerBlock>>>(_ctx.get());
}

static inline __device__
vec3f pixelRayDirection(const Context* ctx, int x, int y)
{
    vec3f P
    {
        ctx->topLeftCorner.x + ctx->half_dx * (2*x),
        ctx->topLeftCorner.y - ctx->half_dy * (2*y),
        -1.0f
    };
    P = ctx->cameraToWorld.transform3x4(P);
    return (P - ctx->cameraPosition).versor();
}

__global__
void render(Context* ctx)
{
    auto i = blockIdx.x * blockDim.x + threadIdx.x;
    auto j = blockIdx.y * blockDim.y + threadIdx.y;
    auto d = pixelRayDirection(ctx, i, j);
    Ray pixelRay
    {
        .origin = ctx->cameraPosition,
        .direction = d,
        .tMin = 0.001f,
        .tMax = cuda::std::numeric_limits<float>::infinity()
    };
    vec3f c = trace(ctx, pixelRay, vec3f(1), ctx->options.recursionDepth);
    c.x = math::clamp(c.x, 0.0f, 1.0f);
    c.y = math::clamp(c.y, 0.0f, 1.0f);
    c.z = math::clamp(c.z, 0.0f, 1.0f);
    ctx->frame->at(i, j) = pack_sRGB(c.x, c.y, c.z);
}

__device__
vec3f trace(const Context* ctx, const Ray& ray, vec3f attenuation, int depth)
{
    Intersection hit
    {
        .object = nullptr,
        .t = ray.tMax,
    };
    int nearestObject = intersect(ctx, hit, ray);

    if (nearestObject < 0)
        return miss(ctx);

    // Is opaque?
    // auto& m = _scene->objects.get<Key::eMaterial>(nearestObject);
    // if (m.opacity < 0.999f)
    // {
    //     // anyHit(ctx);
    // }

    return closestHit(ctx, hit, ray, attenuation, nearestObject, depth);
}

__device__
int intersect(const Context*, Intersection&, const Ray&)
{

}

__device__
vec3f miss(const Context* ctx)
{

}

__device__
vec3f anyHit(const Context* ctx)
{

}

__device__
vec3f closestHit(const Context* ctx, const Intersection& hit, const Ray& ray,
    vec3f attenuation, uint32_t object, int depth)
{

}

} // namespace cg::rt
