#pragma once

#include <math/Vector4.h>
#include "BVH.h"
#include "Frame.h"
#include "Scene.h"
#include "../Ref.h"

// #define USE_MONTECARLO_SAMPLING

namespace cg::rt
{

enum struct Counter
{
    Threads = 0,
    Rays = 1,
    ShadowRays,
};

class RayTracer : public SharedObject
{
public:
    static constexpr float c_MinRadiance = 0x1p-8f;

    struct Stats
    {
        uint32_t rays = 0;
        uint32_t hits = 0;
        uint32_t shadowRays = 0;
        uint32_t shadowHits = 0;
    };

    struct Options
    {
        int         samples = 8;
        float       eps = 1e-4f;
        bool        flipYAxis = false;
        int         recursionDepth = 6;
        int         device = 0; // CUDA device id
        Frame*      heatMap = nullptr;
    };

    struct Context;

    RayTracer();
    RayTracer(Options&& op) : RayTracer() { options = std::move(op); }
    ~RayTracer() override;

    void render(Frame* frame, const Camera* camera, const Scene* scene,
        cudaStream_t stream = 0);

    bool running() const
    {
        return cudaEventQuery(finished) == cudaErrorNotReady;
    }

    // Ray tracer options.
    // Always check if running() == false of `finished` event member
    // writing to it.
    Options options;
    cudaEvent_t started, finished;

private:
    Context* _ctx {nullptr};
    void* _pRandomStates {nullptr};
};

extern __managed__ RayTracer::Stats g_RTStats;

} // namespace cg::rt
