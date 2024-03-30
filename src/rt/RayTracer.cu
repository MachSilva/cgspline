#include "RayTracer.h"

#include <geometry/Bounds3.h>

namespace cg::rt
{

template<typename T>
struct HashTable
{
    __host__ __device__
    constexpr T* get(uint32_t key)
    {

    }

    __host__ __device__
    constexpr auto slot(uint32_t key)
    {

    }

protected:
    uint16_t slotCount;
    uint16_t elementCount;
    T* data;
};

__global__
void trace(Frame* frame, const Camera* camera, const Scene* scene)
{

}

__device__
vec3f miss()
{

}

__device__
vec3f anyHit()
{

}

__device__
vec3f closestHit()
{

}

__device__
bool intersect()
{

}

} // namespace cg::rt
