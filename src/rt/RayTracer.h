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
        int recursionDepth = 5;
        int diffusionRays = 1;
    };

    RayTracer() = default;
    RayTracer(Options&& op) : _options{std::move(op)} {}

    void render(Frame* frame, const Camera* camera, const Scene* scene);

    const auto& options() const { return _options; }
    void setOptions(Options&& op);

private:
    Options _options {};
    Frame* _frame = nullptr;
    Scene* _scene = nullptr;
};

__global__ void render(Frame* frame, const Camera* camera, const Scene* scene);

__device__ vec3f trace();

__device__ vec3f miss();

__device__ vec3f anyHit();

__device__ vec3f closestHit();

__device__ bool intersect();

} // namespace cg::rt
