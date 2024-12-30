#pragma once

#include <math/Vector4.h>
#include "BVH.h"
#include "Frame.h"
#include "Scene.h"
#include "../Ref.h"

namespace cg::rt
{

class RayTracer : public SharedObject
{
public:
    struct Options
    {
        int         diffusionRays = 1;
        float       eps = 1e-4f;
        bool        flipYAxis = false;
        int         recursionDepth = 6;
        int         device = 0; // CUDA device id
        Frame*      heatMap = nullptr;
    };

    struct Context;

    RayTracer();
    RayTracer(const Options& op) : RayTracer() { _options = op; }
    ~RayTracer() override;

    void render(Frame* frame, const Camera* camera, const Scene* scene,
        cudaStream_t stream = 0);

    const auto& options() const { return _options; }

    cudaEvent_t started, finished;
private:
    Options _options {};
    Context* _ctx {nullptr};
};

} // namespace cg::rt
