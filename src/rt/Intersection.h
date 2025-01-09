#pragma once

#include <cinttypes>
#include "../AlignedTypes.h"

namespace cg::rt
{

using spline::vec2;
using spline::vec3;
using spline::vec4;
using spline::mat3;
using spline::mat4;
using spline::quat;

struct Ray
{
    vec3 origin;
    vec3 direction;
    // vec3 up;
    // vec3 left;
    float max; // max range
    // float ior = 1.0f; //< medium index of refraction
};

// constexpr auto ____s = sizeof (Ray);

struct Intersection
{
    const void* object; // hit object pointer
    vec3 coordinates;   // element coordinates; barycentric coordinates for triangles
    uint32_t index;     // element index; triangle index; patch index...
    float t;            // ray position of the hit
};

} // namespace cg::rt
