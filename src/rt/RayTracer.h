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
        vec3f       backgroundColor = {0.1, 0.1, 0.1};
        int         diffusionRays = 1;
        float       eps = 1e-4f;
        bool        flipYAxis = false;
        int         recursionDepth = 6;
        int         device = 0; // CUDA device id
        Frame*      heatMap = nullptr;
    };

    struct Context;

    RayTracer() = default;
    RayTracer(const Options& op) : _options{op} {}

    void render(Frame* frame, const Camera* camera, const Scene* scene,
        cudaStream_t stream = 0);

    const auto& options() const { return _options; }
    // void setOptions(const Options& op) {  }

private:
    Options _options {};
    std::unique_ptr<Context, void(*)(void*)> _ctx {nullptr, nullptr};
};

} // namespace cg::rt
