#include "RayTracer.h"

#include <graphics/Color.h>
#include <geometry/Bounds3.h>
#include <cmath>
#include <utility>
#include <math_constants.h>
#include <cuda_gl_interop.h>
#include <curand_kernel.h>
#include "PBR.h"
#include "../Log.h"

namespace cg::rt
{

__managed__ Frame* g_BezierClippingHeatMap;

using curand_state_type = curandStatePhilox4_32_10_t;
constexpr int c_RandomGridSize = 256;
constexpr bool c_LoadRandomState = true;

struct RayPayload
{
    curand_state_type random;
    Ray ray;
    vec3 color;
    vec3 attenuation;
    int depth;
    // float dxdy;
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
    curand_state_type* pRandomStates;

    __device__ bool anyHit(RayPayload&) const;
    __device__ bool closestHit(const Intersection&, RayPayload&, uint32_t object) const;
    __device__ bool intersect(const Ray&) const;
    __device__ int  intersect(Intersection&, const Ray&) const;
    __device__ bool miss(RayPayload&) const;
    __device__ bool trace(RayPayload&) const;

    __device__
    vec3 pixelRayDirection(float x, float y) const
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
__global__ void random_state_init(curand_state_type*);

RayTracer::RayTracer()
{
    CUDA_CHECK(cudaMalloc(&_ctx, sizeof (Context)));
    CUDA_CHECK(cudaFuncSetCacheConfig(&rt::render, cudaFuncCachePreferL1));
    CUDA_CHECK(cudaEventCreate(&started));
    CUDA_CHECK(cudaEventCreate(&finished));
    CUDA_CHECK(cudaMalloc(&_pRandomStates,
        squared(c_RandomGridSize) * sizeof (curand_state_type)));

    constexpr dim3 blockDim (32, 32);
    constexpr dim3 gridDim
    {
        c_RandomGridSize / blockDim.x,
        c_RandomGridSize / blockDim.y
    };
    random_state_init<<<gridDim,blockDim>>>((curand_state_type*)_pRandomStates);
    CUDA_CHECK(cudaStreamSynchronize(0));
}

RayTracer::~RayTracer()
{
    CUDA_CHECK_NOEXCEPT(cudaEventDestroy(finished));
    CUDA_CHECK_NOEXCEPT(cudaEventDestroy(started));
    CUDA_CHECK_NOEXCEPT(cudaFree(_pRandomStates));
    CUDA_CHECK_NOEXCEPT(cudaFree(_ctx));
}

void RayTracer::render(Frame* frame, const Camera* camera, const Scene* scene,
    cudaStream_t stream)
{
    auto q = cudaEventQuery(this->finished);
    if (q == cudaErrorNotReady)
    {
        log::warn("CUDA ray tracer not ready: wait the previous work to finish"
            " or create a new instance");
        return;
    }
    CUDA_CHECK(q);
    CUDA_CHECK(cudaEventRecord(this->started, stream));
    CUDA_CHECK(cudaMemPrefetchAsync(frame->data(), frame->size_bytes(),
        options.device, stream));

    auto& objs = scene->objects;
    std::apply(
        [device=options.device,n=objs.size(),stream]
        (auto*... ptrs) -> void
        {
            ((CUDA_CHECK(cudaMemPrefetchAsync(ptrs,
                n * sizeof (decltype(*ptrs)), device, stream))), ...);
        },
        objs.as_pointer_tuple()
    );

    uint32_t w = frame->width();
    uint32_t h = frame->height();

    g_BezierClippingHeatMap = options.heatMap;

    {
        Context ctx;
        ctx.options = options;
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

        if (options.flipYAxis)
        {
            ctx.topLeftCorner.y = -ctx.topLeftCorner.y;
            ctx.half_dy = -ctx.half_dy;
        }

        ctx.pRandomStates = (curand_state_type*)_pRandomStates;

        CUDA_CHECK(cudaMemcpyAsync(_ctx, &ctx, sizeof (ctx),
            cudaMemcpyHostToDevice, stream));
    }

    constexpr dim3 threadsPerBlock (8, 16);
    dim3 blocksPerGrid
    {
        (w / threadsPerBlock.x) + (w % threadsPerBlock.x ? 1 : 0),
        (h / threadsPerBlock.y) + (h % threadsPerBlock.y ? 1 : 0)
    };

    rt::render<<<blocksPerGrid, threadsPerBlock, 0, stream>>>(_ctx);
    CUDA_CHECK(cudaEventRecord(this->finished, stream));
}

__global__
void random_state_init(curand_state_type* p)
{
    constexpr auto n = c_RandomGridSize;
    auto i = blockIdx.x * blockDim.x + threadIdx.x;
    auto j = blockIdx.y * blockDim.y + threadIdx.y;
    if (i >= n || j >= n)
        return;
    auto k = i + j*n;
    curand_init(42, k, 0, p+k);
}

__global__ //__launch_bounds__(64)
void render(Context* ctx)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    if (i >= ctx->frame->width() || j >= ctx->frame->height())
        return;

    RayPayload payload;
    payload.color = vec3(0);
    // {
    //     .ray = {
    //         .origin = ctx->cameraPosition,
    //         // .direction = d,
    //         .max = numeric_limits<float>::infinity()
    //     },
    //     .color = vec3(0),
    //     // .attenuation = vec3(1),
    //     // .depth = ctx->options.recursionDepth
    // };

    if constexpr (c_LoadRandomState)
    { // Load random state
        int a = i % c_RandomGridSize;
        int b = j % c_RandomGridSize;
        payload.random = ctx->pRandomStates[a + b*c_RandomGridSize];
    }
    else
    {
        curand_init(42, i + (j << 10), 0, &payload.random);
    }

    int n = ctx->options.samples;
    float n_inv = 1.0f / n;
    for (int s = 0; s < n; s++)
    {
        float s0 = s * n_inv;
        float s1 = vdc(s);
        payload = {
            .random = payload.random,
            .ray = {
                .origin = ctx->cameraPosition,
                .direction = ctx->pixelRayDirection(i + s0, j + s1),
                .max = numeric_limits<float>::infinity()
            },
            .color = payload.color,
            .attenuation = vec3(1),
            .depth = ctx->options.recursionDepth,
        };

        while (ctx->trace(payload));
    }
    vec3 c = payload.color * n_inv;
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
        .t = payload.ray.max,
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
        localRay.max = ray.max * d;
        d = 1 / d;
        localRay.direction *= d;
        localHit.t = localRay.max;

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
        localRay.max = ray.max * d;
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
    vec3 color {0};

    // From the point to the camera; BRDF uses this vector orientation.
    vec3 V = - payload.ray.direction;

    vec3 N;
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
    float dotNV = vec3::dot(N, V);

    if (dotNV < 0)
    {
        // backfaced = true;
        N = -N;
        dotNV = -dotNV;
    }

    vec3 P = payload.ray.origin + hit.t * payload.ray.direction;

    const auto& lights = this->scene->lights;
    for (int i = 0; i < lights.size(); i++)
    {
        vec3 I;
        vec3 L;
        float d;

        if (!lightVector(d, L, P, lights[i]))
            continue;

        const float dotNL = vec3::dot(N, L);
        if (dotNL < 1e-14f) // Avoid division by zero and backfaced lights
            continue;

        // Shadow
        Ray r1 { .origin = P + this->options.eps * L, .direction = L, .max = d };
        if (intersect(r1))
            continue;

        // I = lightColor(d, lights[i]);
        // color += I * BRDF(L, V, N, dotNV, dotNL, m);
        vec3 H = (L + V).versor();
        float dotHN = H.dot(N);
        float dotHL = H.dot(L);

        I = lightColor(d, lights[i]);
        color += I * mix(
            BRDF_diffuse(m),
            BRDF_specular(dotHL, dotHN, dotNV, dotNL, m.roughness, m.specular),
            m.metalness
        );
    }

    payload.color += color * std::numbers::pi_v<float> * payload.attenuation;

    if (payload.depth <= 0)
        return false;

    // Select random microfacet normal
    vec3 M;
    {
        vec2 c = microfacet(
            squared(m.roughness),
            curand_uniform(&payload.random),
            curand_uniform(&payload.random)
        );
        vec3 t_s = N.cross(V).versor(); // tangent perpendicular
        vec3 t_p = N.cross(t_s).versor(); // tangent parallel
        M = sinf(c.x)*cosf(c.y)*t_s + sinf(c.x)*sinf(c.y)*t_p + cosf(c.x)*N;

        // quat q (vec3(0,0,1).cross(N), 1 + N.z);
        // q.normalize();
        // M.x = sinf(c.x)*cosf(c.y);
        // M.y = sinf(c.x)*sinf(c.y);
        // M.z = cosf(c.x);
        // M = q.rotate(M);
        M.normalize();
    }
    float dotMV = M.dot(V);

    // Reflection
    vec3 reflectance = schlick(m.specular, dotMV);
    vec3 R = reflect(-V, M, -dotMV);
    float dotMN = M.dot(N);
    float dotNR = N.dot(R);
    if (dotNR < 1e-14f)
        return false;

    float g = G(dotNR, dotNV, m.roughness) * dotMV / (dotNV * dotMN);
    vec3 brdf = mix(
        BRDF_diffuse(m),
        vec3(reflectance * g),
        m.metalness
    );
    vec3 a = payload.attenuation * brdf;
    if (a.max() <= c_MinRadiance)
        return false;

    payload.ray =
    {
        .origin = P + options.eps * R,
        .direction = R,
        .max = numeric_limits<float>::infinity()
    };
    payload.attenuation = a;
    payload.depth = payload.depth - 1;
    return true;
}

} // namespace cg::rt
