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
    };

    struct __align__(8) Context
    {
        vec2f topLeftCorner;
        float half_dx;
        float half_dy;
        mat4f cameraToWorld;
        mat3f cameraNormalToWorld;
        vec3f cameraPosition;
        uint32_t height;
        uint32_t width;
        __align__(8) Raw<Frame> frame;
        __align__(8) Raw<const Scene> scene;
        __align__(8) Options options;
    };

    RayTracer() = default;
    RayTracer(Options&& op) : _options{std::move(op)} {}

    void render(Frame* frame, const Camera* camera, const Scene* scene);

    const auto& options() const { return _options; }
    void setOptions(Options&& op);

private:
    using Key = Scene::Key;

    Options _options {};
    std::unique_ptr<Context, void(*)(void*)> _ctx {nullptr, nullptr};
};

} // namespace cg::rt
