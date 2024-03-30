#pragma once

#include <cinttypes>
#include <math/Vector4.h>

namespace cg::rt
{

struct Ray
{
    vec3f origin;
    vec3f direction;
    // vec3f up;
    // vec3f left;
    float tMin;
    float tMax;
    float mediumRefractionIndex = 1.0f;
};

struct Intersection
{
    const void* object; // hit object pointer
    vec3f coordinates;  // element coordinates; barycentric coordinates for triangles
    uint32_t index;     // element index; triangle index; patch index...
    float t;            // ray position of the hit
};

} // namespace cg::rt
