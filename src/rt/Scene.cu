#include "Scene.h"

namespace cg::rt
{

mat4f Transform::matrix() const
{
    return mat4f::TRS(position, rotation, scale);
}

mat4f Transform::inverseMatrix() const
{
    mat3f R {rotation};
    R[0] *= (1.0f / scale.x);
    R[1] *= (1.0f / scale.y);
    R[2] *= (1.0f / scale.z);
    // nvcc:
    // internal error: assertion failed at: "interpret.c", line 2607 in get_runtime_array_pos
    //           { R[0][1], R[1][1], R[2][1], 0 },
    //                          ^
    return mat4f
    { // columns
        { R[0][0], R[1][0], R[2][0], 0 },
        { R[0][1], R[1].y , R[2][1], 0 },
        { R[0][2], R[1].z , R[2][2], 0 },
        { -position.dot(R[0]), -position.dot(R[1]), -position.dot(R[2]), 1 }
    };
}

mat3f Transform::normalMatrix() const
{
    mat3f R {rotation};
    R[0] *= (1.0f / scale.x);
    R[1] *= (1.0f / scale.y);
    R[2] *= (1.0f / scale.z);
    return R;
}

void Scene::buildBVH()
{
    auto n = objects.size();
    Buffer<BVH::ElementData> data (n, objects.resource());
    for (int i = 0; i < n; i++)
    {
        auto& e = data[i];
        e.index = i;
        e.bounds = objects.get<Key::ePrimitive>(i)->bounds(
            objects.get<Key::eLocal2WorldMatrix>(i)
        );
        e.centroid = e.bounds.center();
    }
    topLevelBVH.build(data, 1);
}

} // namespace cg::rt
