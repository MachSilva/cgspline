#pragma once

#include <bit>
#include <functional>
#include <ranges>
#include <cuda/std/cmath>
#include <geometry/Bounds3.h>
#include "Intersection.h"
#include "PerfectHashFunction.h"
#include "RTNamespace.h"

namespace cg::rt
{

using key_t = uint32_t;

struct BVH
{
    static constexpr int MAX_BITS = 8 * sizeof (key_t);
    static constexpr auto EMPTY = key_t(-1);
    static constexpr auto ROOT_KEY = key_t(1);

    struct __align__(16) Node
    {
        Bounds3f leftBox;
        Bounds3f rightBox;
        uint32_t left; // Node offset at the array (NOT the key)
        union
        {
            uint32_t right; // Node offset at the array (NOT the key)
            uint32_t first;
        };
        // Uncle must exist for all non-leaf non-root nodes
        // Node offset at the array (NOT the key)
        // uint32_t uncle;

        __host__ __device__
        bool isLeaf() const noexcept { return left == EMPTY; }
    };

    struct ElementData
    {
        Bounds3f bounds;
        vec3 centroid;
        uint32_t index;
    };

    BVH(memory_resource* mr = ManagedResource::instance())
        : _alloc{mr}, _keys{mr}, _nodes{mr}, _indices{mr}, _table{mr}
    {}

    constexpr auto root() const { return 0; }

    std::span<const Node>     nodes() const noexcept { return _nodes; }
    std::span<Node>           nodes() noexcept { return _nodes; }
    std::span<const uint32_t> indices() const noexcept { return _indices; }
    std::span<const key_t>    keys() const noexcept { return _keys; }

    void build(std::span<ElementData> elements, uint32_t elementsPerNode = 1);

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

    __host__ __device__
    bool hashIntersect(Intersection& hit, const Ray&,
        std::predicate<Intersection&,const Ray&,uint32_t> auto intersectfn) const;

    __host__ __device__
    bool hashIntersect(const Ray&,
        std::predicate<const Ray&,uint32_t> auto intersectfn) const;

private:
    polymorphic_allocator<Node> _alloc;
    uint32_t _elementsPerNode = 1;

    PerfectHashFunction _hash;

    Array<Node> _nodes;
    Array<key_t> _keys;
    Array<uint32_t> _indices;
    Array<uint32_t> _table;

    void link(Node* node, key_t sibling);
    key_t split(std::span<ElementData> elements, key_t key);
    key_t wrap(std::span<ElementData> elements, key_t key);
};

namespace binarytree
{


template<typename T>
__host__ __device__
constexpr T parent(T node) { return node >> 1U; }

template<typename T>
__host__ __device__
constexpr T sibling(T node) { return node ^ 1U; }

template<typename T>
__host__ __device__
constexpr T left(T node) { return node << 1U; }

template<typename T>
__host__ __device__
constexpr T right(T node) { return sibling(left(node)); }

template<typename T>
__host__ __device__
constexpr T uncle(T node) { return sibling(parent(node)); }

} // namespace binarytree

/**
 * @brief Count trailing zeros
 */
static inline __host__ __device__
int ctz(uint32_t x) noexcept
{
#if !defined(__CUDA_ARCH__)
    return std::countr_zero(x);
#else
    return __clz(__brev(x));
#endif
}

static inline __host__ __device__
int ctz(uint64_t x) noexcept
{
#if !defined(__CUDA_ARCH__)
    return std::countr_zero(x);
#else
    return __clzll(__brevll(x));
#endif
}

static inline
__host__ __device__
bool _node_intersect(float& tMin, float& tMax, const Bounds3f& bounds,
    const vec3& origin, const vec3& directionInverse)
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

__host__ __device__
bool BVH::hashIntersect(Intersection& hit, const Ray& ray,
    std::predicate<Intersection&,const Ray&,uint32_t> auto intersectfn) const
{
    const vec3 D_1 = ray.direction.inverse();
    const Node* node = _nodes.data();
    key_t key = BVH::ROOT_KEY;
    key_t postponed = 0;
    int bits = 0;
    bool result = false;

    while (true)
    {
        assert(bits < MAX_BITS && "BVH intersect max depth achieved");

        if (!node->isLeaf())
        {
            float L0, L1;
            float R0, R1;
            bool iL = _node_intersect(L0, L1, node->leftBox, ray.origin, D_1)
                && L0 < hit.t;
            bool iR = _node_intersect(R0, R1, node->rightBox, ray.origin, D_1)
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
                    if (L0 < R0)
                    {
                        key = binarytree::left(key);
                        node = _nodes.data() + node->left;
                    }
                    else
                    {
                        key = binarytree::right(key);
                        node = _nodes.data() + node->right;
                    }
                }
                else // only left
                {
                    key = binarytree::left(key);
                    node = _nodes.data() + node->left;
                }
                continue;
            }
            else if (iR) // only right
            {
                key = binarytree::right(key);
                node = _nodes.data() + node->right;
                continue;
            }
        }
        else // is a leaf
        {
            auto i = node->first;
            while (_indices[i] != EMPTY)
            {
                bool b = intersectfn(hit, ray, _indices[i++]);
                result |= b;
            }
        }

        // going up
        if (!postponed)
            break;

        int levels = ctz(postponed);
        postponed >>= levels;
        bits = bits - levels;
        key = binarytree::sibling(key >> levels);
        postponed ^= 1;
        auto d_hash = _hash(key);
        assert(d_hash < _table.size());
        auto d_offset = _table[d_hash];
        assert(d_offset < _nodes.size());
        node = _nodes.data() + _table[_hash(key)];
    }

    return result;
}

__host__ __device__
bool BVH::hashIntersect(const Ray& ray,
    std::predicate<const Ray&,uint32_t> auto intersectfn) const
{
    const vec3 D_1 = ray.direction.inverse();
    const Node* node = _nodes.data();
    key_t key = BVH::ROOT_KEY;
    key_t postponed = 0;
    int bits = 0;

    while (true)
    {
        assert(bits < MAX_BITS && "BVH intersect max depth achieved");
        if (!node->isLeaf())
        {
            float L0, L1;
            float R0, R1;
            bool iL = _node_intersect(L0, L1, node->leftBox, ray.origin, D_1)
                && L0 < ray.max;
            bool iR = _node_intersect(R0, R1, node->rightBox, ray.origin, D_1)
                && R0 < ray.max;

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
                    if (L0 < R0)
                    {
                        key = binarytree::left(key);
                        node = _nodes.data() + node->left;
                    }
                    else
                    {
                        key = binarytree::right(key);
                        node = _nodes.data() + node->right;
                    }
                }
                else // only left
                {
                    key = binarytree::left(key);
                    node = _nodes.data() + node->left;
                }
                continue;
            }
            else if (iR) // only right
            {
                key = binarytree::right(key);
                node = _nodes.data() + node->right;
                continue;
            }
        }
        else // is a leaf
        {
            auto i = node->first;
            while (_indices[i] != EMPTY)
            {
                if (intersectfn(ray, _indices[i++]))
                    return true;
            }
        }

        // going up
        if (!postponed)
            break;

        auto levels = ctz(postponed);
        postponed >>= levels;
        bits = bits - levels;
        key = binarytree::sibling(key >> levels);
        postponed ^= 1;
        node = _nodes.data() + _table[_hash(key)];
    }

    return false;
}

} // namespace cg::rt
