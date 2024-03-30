#pragma once

#include <math/Matrix4x4.h>
#include <cuda/std/span>
#include <concepts>
#include <forward_list>
#include <list>
#include <numbers>
#include "BVH.h"
#include "Primitive.h"
#include "SoA.h"

namespace cg::rt
{

struct __align__(4) Transform
{
    [[nodiscard]] mat4f matrix() const;
    [[nodiscard]] mat4f inverseMatrix() const;
    [[nodiscard]] mat3f normalMatrix() const;

    quatf rotation = quatf::identity();
    vec3f position {0};
    float __padding0;
    vec3f scale {1};
    float __padding1;
};

struct Camera
{
    constexpr static vec3f viewForward {0,0,1};
    constexpr static vec3f viewUp {0,1,0};

    const auto& position() const { return transform.position; }

    Transform transform;
    float aspectRatio;
    float fieldOfView;
    float height;
    enum ProjectionType
    {
        Perspective,
        Parallel
    } projection;
};

struct __align__(4) Light
{
    // Radiance for each primary wavelength.
    vec3f color;
    float strength;
    vec3f position;
    float range; // range (== 0 INFINITE)
    vec3f direction;
    float angle; // spot light angle (if < 0.01: directional; if > 1.57: point)

    enum Type
    {
        Directional = 0, Point = 1, Spot = 2
    };

    __host__ __device__
    Type type() const
    {
        if (isDirectional())
            return Directional;
        if (isPoint()) // > ~179° (1.57 rad)
            return Point;
        return Spot;
    }

    __host__ __device__
    bool isDirectional() const { return angle < 0.01f; }

    __host__ __device__
    bool isPoint() const { return angle > 1.57f; } // > ~179° (1.57 rad)
};

struct __align__(4) Material
{
    vec3f diffuse;
    // Spectral reflectance distribution for use with Schilick's approximation
    vec3f specular;
    vec3f transparency;
    float metalness; // [0,1]
    float roughness; // [0,1]
    float refractiveIndex;
};

struct Scene
{
    enum Key
    {
        eLocal2WorldMatrix,
        eWorld2LocalMatrix,
        eTransform,
        eMaterial,
        ePrimitive,
    };

    using ObjectArrays = SoA<uint32_t,mat4f,mat4f,Transform,Material,Primitive*>;

    ObjectArrays objects;
    vector<Light> lights {};
    vector<Mesh> meshes {};
    vector<BezierSurface> surfaces {};
    vector<BVH> bvhs {};
    BVH topLevelBVH {};

    Scene() = default;
    Scene(uint32_t capacity, std::pmr::memory_resource* mr)
        : objects{capacity, mr}, lights{mr}, meshes{mr}, surfaces{mr}
        , bvhs{mr}, topLevelBVH{mr} {}

    ~Scene()
    {
        for (auto& e : _allocations)
        {
            resource()->deallocate(e.ptr, e.len, e.align);
        }
    }

    template<typename T>
    T* createBuffer(uint32_t n)
    {
        uint32_t len = n * sizeof (T);
        void* p = resource()->allocate(len, alignof (T));
        _allocations.push_front({ .ptr = p, .len = len, .align = alignof (T)});
        return (T*) p;
    }

    std::pmr::memory_resource* resource() const
    {
        return objects.resource();
    }

    void buildBVH();

private:
    struct Alloc
    {
        void* ptr;
        uint32_t len;
        uint32_t align;
    };

    std::forward_list<Alloc> _allocations;
};

struct SceneObjectRef
{
    Scene* _scene;
    uint32_t objectIdx;
};

struct RefractionIndex
{
    static constexpr float Vacuum = 1.0;
    static constexpr float AirAtSeaLevel = 1.00029;
    static constexpr float Ice = 1.31;
    static constexpr float Water = 1.333;
    static constexpr float FusedQuartz = 1.46;
    static constexpr float Glass = 1.55;
    static constexpr float Sapphire = 1.77;
    static constexpr float Diamond = 2.42;
};

} // namespace cg::rt
