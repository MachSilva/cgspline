#pragma once

#include <bit>
#include <deque>
#include <functional>
#include <limits>
#include <memory_resource>
#include <numeric>
#include <ranges>
#include <geometry/Bounds3.h>
#include <cuda/std/bit>
#include <thrust/universal_vector.h>
#include "Intersection.h"
#include "PerfectHashFunction.h"
#include "RTNamespace.h"

namespace cg::rt
{

// template<typename F>
// concept intersect_nearest_invocable = requires (F f)
// {

// };

struct BVH
{
    static constexpr auto EMPTY = uint32_t(-1);
    static constexpr uint32_t ROOT_KEY = 1;

    struct __align__(8) Node
    {
        Bounds3f leftBox;
        Bounds3f rightBox;
        uint32_t left; // Node offset at the array (NOT the key)
        union
        {
            uint32_t right; // Node offset at the array (NOT the key)
            struct
            {
                uint32_t first;
                uint32_t count;
            };
        };
        // Uncle must exist for all non-leaf non-root nodes
        // Node offset at the array (NOT the key)
        uint32_t uncle;

        __host__ __device__
        bool isLeaf() const noexcept { return left == EMPTY; }
    };

    struct ElementData
    {
        Bounds3f bounds;
        vec3f centroid;
        uint32_t index;
    };

    BVH(const polymorphic_allocator<Node>& alloc = {})
        : _alloc{alloc}, _keys{alloc}, _nodes{alloc}, _indices{alloc}
    {}

    const auto root() const noexcept { return _root; }

    span<const Node> nodes() const noexcept
    {
        return {_nodes.data(), _nodes.size()};
    }

    span<Node> nodes() noexcept
    {
        return {_nodes.data(), _nodes.size()};
    }

    span<const uint32_t> indices() const noexcept
    {
        return {_indices.data(), _indices.size()};
    }

    span<const uint32_t> keys() const noexcept
    {
        return {_keys.data(), _keys.size()};
    }

    void build(span<ElementData> elements, uint32_t elementsPerNode = 1);

    void buildHashTable(cudaStream_t stream = 0);

    bool intersect(
        Intersection& hit,
        const Ray& ray,
        std::function<bool(Intersection&, const Ray&, uint32_t index)>
            intersectfn) const;

    bool intersect(
        const Ray& ray,
        std::function<bool(const Ray&, uint32_t index)>
            intersectfn) const;

    template<std::invocable<Intersection&,const Ray&,uint32_t> Fn>
    __host__ __device__
    bool hashIntersect(Intersection& hit, const Ray&, Fn intersectfn) const;

    template<std::regular_invocable<const Ray&,uint32_t> Fn>
    __host__ __device__
    bool hashIntersect(const Ray&, Fn intersectfn) const;

private:
    polymorphic_allocator<Node> _alloc;
    vector<uint32_t> _keys;
    vector<Node> _nodes;
    vector<uint32_t> _indices;
    uint32_t _elementsPerNode = 1;
    uint32_t _root = EMPTY;

    PerfectHashFunction _hash;
    ManagedBuffer<uint32_t> _table;

    void link(Node* node, uint32_t sibling);
    uint32_t split(span<ElementData> elements, uint32_t key);
    uint32_t wrap(span<ElementData> elements, uint32_t key);
};

namespace binarytree
{

constexpr uint32_t parent(uint32_t node) { return node >> 1U; }

constexpr uint32_t sibling(uint32_t node) { return node ^ 1U; }

constexpr uint32_t left(uint32_t node) { return node << 1U; }

constexpr uint32_t right(uint32_t node) { return sibling(left(node)); }

constexpr uint32_t uncle(uint32_t node) { return sibling(parent(node)); }

} // namespace binarytree

static inline
__host__ __device__
bool _node_intersect(float& tMin, float& tMax, const Bounds3f& bounds,
    const vec3f& origin, const vec3f& directionInverse)
{
    using flt = numeric_limits<float>;
    static_assert(flt::is_iec559, "Quiet NaNs and infinities required");

    auto& b0 = bounds.min();
    auto& b1 = bounds.max();
    tMin = -flt::infinity();
    tMax = +flt::infinity();

    // y = A + Bt
    // t = (y - A) / B, B != 0
    #pragma unroll
    for (int i = 0; i < 3; i++)
    {
        int s = std::signbit(directionInverse[i]);
        float t[2];
        t[0] = (b0[i] - origin[i]) * directionInverse[i];
        t[1] = (b1[i] - origin[i]) * directionInverse[i];
        if (t[0 + s] > tMin)
            tMin = t[0 + s];
        if (t[1 - s] < tMax)
            tMax = t[1 - s];
    }

    return tMin < tMax && tMax > 0.0001f;
}

template<std::invocable<Intersection&,const Ray&,uint32_t> Fn>
__host__ __device__
bool BVH::hashIntersect(Intersection& hit, const Ray& ray,
    Fn intersectfn) const
{
    if (_root == BVH::EMPTY)
        return false;

    const vec3f D_1 = ray.direction.inverse();
    const Node* node = &_nodes[_root];
    uint32_t key = BVH::ROOT_KEY;
    uint32_t postponed = 0;
    int bits = 0;
    bool result = false;

    while (true)
    {
        assert(bits >= 0 && bits < 32 && "BVH intersect max depth achieved");

        if (!node->isLeaf())
        {
            float L0, L1;
            float R0, R1;
            bool iL = _node_intersect(L0, L1, node->leftBox, ray.origin, D_1)
                && L0 < hit.t;
            bool iR = node->right != BVH::EMPTY
                && _node_intersect(R0, R1, node->rightBox, ray.origin, D_1)
                && R0 < hit.t;

            if (iL || iR)
            {
                postponed <<= 1;
                bits++;
            }

            // going down
            if (iL)
            {
                if (iR)
                {
                    postponed |= 1; // postpone a node
                    if (L0 < L1)
                    {
                        key = binarytree::left(key);
                        node = &_nodes[node->left];
                    }
                    else
                    {
                        key = binarytree::right(key);
                        node = &_nodes[node->right];
                    }
                }
                else // only left
                {
                    key = binarytree::left(key);
                    node = &_nodes[node->left];
                }
                continue;
            }
            else if (iR) // only right
            {
                key = binarytree::right(key);
                node = &_nodes[node->right];
                continue;
            }
        }
        else // is a leaf
        {
            auto base = node->first;
            for (int i = 0; i < node->count; i++)
            {
                bool b = intersectfn(hit, ray, _indices[base + i]);
                result |= b;
            }
        }

        // going up
        if (postponed & 1)
        {
            postponed ^= 1;
            key = binarytree::sibling(key);
            node = &_nodes[_table[_hash(key)]];
            continue;
        }
        else if (postponed & 2)
        {
            postponed >>= 1;
            postponed ^= 1;
            key = binarytree::uncle(key);
            assert(node->uncle < _nodes.size());
            node = &_nodes[node->uncle];
            continue;
        }

        if (!postponed)
            break;

        auto levels = ::cuda::std::countr_zero(postponed);
        postponed >>= levels;
        bits = bits - levels;
        key = binarytree::sibling(key >> levels);
        postponed ^= 1;
        // assert(_table[_hash(key)] < _nodes.size());
        node = &_nodes[_table[_hash(key)]];
    }

    return result;
}


template<std::regular_invocable<const Ray&,uint32_t> Fn>
__host__ __device__
bool BVH::hashIntersect(const Ray& ray, Fn intersectfn) const
{
    if (_root == BVH::EMPTY)
        return false;

    const vec3f D_1 = ray.direction.inverse();
    const Node* node = &_nodes[_root];
    uint32_t key = BVH::ROOT_KEY;
    uint32_t postponed = 0;
    int bits = 0;

    while (true)
    {
        assert(bits >= 0 && bits < 32 && "BVH intersect max depth achieved");
        if (!node->isLeaf())
        {
            float L0, L1;
            float R0, R1;
            bool iL = _node_intersect(L0, L1, node->leftBox, ray.origin, D_1)
                && L0 < ray.tMax;
            bool iR = node->right != BVH::EMPTY
                && _node_intersect(R0, R1, node->rightBox, ray.origin, D_1)
                && R0 < ray.tMax;

            if (iL || iR)
            {
                postponed <<= 1;
                bits++;
            }

            // going down
            if (iL)
            {
                if (iR)
                {
                    postponed |= 1; // postpone a node
                    if (L0 < L1)
                    {
                        key = binarytree::left(key);
                        node = &_nodes[node->left];
                    }
                    else
                    {
                        key = binarytree::right(key);
                        node = &_nodes[node->right];
                    }
                }
                else // only left
                {
                    key = binarytree::left(key);
                    node = &_nodes[node->left];
                }
                continue;
            }
            else if (iR) // only right
            {
                key = binarytree::right(key);
                node = &_nodes[node->right];
                continue;
            }
        }
        else // is a leaf
        {
            auto base = node->first;
            for (int i = 0; i < node->count; i++)
            {
                if (intersectfn(ray, _indices[base + i]))
                    return true;
            }
        }

        // going up
        if (postponed & 1)
        {
            postponed ^= 1;
            key = binarytree::sibling(key);
            node = &_nodes[_table[_hash(key)]];
            continue;
        }
        else if (postponed & 2)
        {
            postponed >>= 1;
            postponed ^= 1;
            key = binarytree::uncle(key);
            assert(node->uncle < _nodes.size());
            node = &_nodes[node->uncle];
            continue;
        }

        if (!postponed)
            break;

        auto levels = ::cuda::std::countr_zero(postponed);
        postponed >>= levels;
        bits = bits - levels;
        key = binarytree::sibling(key >> levels);
        postponed ^= 1;
        // assert(_table[_hash(key)] < _nodes.size());
        node = &_nodes[_table[_hash(key)]];
    }

    return false;
}

} // namespace cg::rt
