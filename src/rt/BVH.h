#pragma once

#include <cassert>
#include <array>
#include <bit>
#include <deque>
#include <functional>
#include <limits>
#include <memory_resource>
#include <numeric>
#include <random>
#include <ranges>
#include <cuda/std/span>
#include <geometry/Bounds3.h>
#include "Intersection.h"
#include "RTNamespace.h"

namespace cg::rt
{

// Random prime numbers between 2^30 and 2^31
static constexpr std::array<int32_t,16> RANDOM_PRIMES31
{
    1933273841, 1439646163, 1293153791, 1150219159,
    1547529853, 1733961953, 1408002091, 1741593757,
    1525947499, 2036288231, 1372157351, 1312347037,
    1946609681, 1433388449, 1442234909, 2018826701,
};

// Random prime numbers between 2^31 and 2^32
static constexpr std::array<uint32_t,16> RANDOM_PRIMES32
{
    4174915957U, 2376772961U, 3641499011U, 2226070283U,
    2963417291U, 4089834317U, 3197253559U, 3312796573U,
    3054610121U, 3994827449U, 3128597239U, 4018580789U,
    2442028279U, 3978086807U, 3211402351U, 2816854373U,
};

struct HashFunction
{

    uint32_t a = 31;
    uint32_t b = 19;
    uint32_t p = RANDOM_PRIMES32[0];

public:
    constexpr HashFunction() noexcept = default;

    constexpr explicit
    HashFunction(uint32_t A, uint32_t B, int prime_seed = 0) noexcept
    {
        p = RANDOM_PRIMES32[prime_seed % RANDOM_PRIMES32.size()];
        a = A % p;
        b = B % p;
        // `a` cannot be zero
        if (a == 0)
            a = 31;
    }

    constexpr uint32_t operator() (uint32_t value) const
    {
        return (a*value + b) % p;
    }
};

class PerfectHashTable
{
    HashFunction _h;
    std::mt19937 _mt;

    vector<uint32_t> _displacement;
    uint32_t _displ_mask;
    uint32_t _table_size;

public:
    PerfectHashTable(uint32_t seed = 42,
        std::pmr::memory_resource* mr = std::pmr::get_default_resource())
        : _displacement{mr}, _mt{seed} {}

    void build(const span<uint32_t> keys);

    uint32_t hash(uint32_t key) const
    {
        return _h(key + _displacement[key & _displ_mask]) % _table_size;
    }
};

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

    bool intersect(
        Intersection& hit,
        const Ray& ray,
        std::function<bool(Intersection&, const Ray&, uint32_t index)>
            intersectElement) const;

    bool intersects(
        const Ray& ray,
        std::function<bool(const Ray&, uint32_t index)>
            intersectsElement) const;

    // template<typename Fn>
    // bool intersect(Intersection& hit, const Ray&, Fn intersectElement) const;

private:
    polymorphic_allocator<Node> _alloc;
    vector<uint32_t> _keys;
    vector<Node> _nodes;
    vector<uint32_t> _indices;
    uint32_t _elementsPerNode = 1;
    uint32_t _root = EMPTY;

    void link(uint32_t node, uint32_t uncle);
    uint32_t split(span<ElementData> elements, uint32_t key);
    uint32_t wrap(span<ElementData> elements, uint32_t key);
};

// template<typename Fn>
// bool BVH::intersect(Intersection& hit, const Ray3f& ray,
//     Fn intersectElement) const
// {
//     if (_root == BVH::EMPTY)
//         return false;

//     const vec3f D_1 = ray.direction.inverse();
//     const Node* node = &_nodes[_root];
//     uint32_t key = BVH::ROOT_KEY;
//     uint32_t postponed = 0;
//     int bits = 1;
//     bool result = false;

//     auto up = [&]()
//     {
//         // has a remaining/postponed node one level above
//         while (postponedBits > 0)
//         {
//             if (node->uncle == BVH::EMPTY)
//             {
//                 node = nullptr;
//                 return;
//             }
//             bool stop = postponed & 1U;
//             node = &_nodes[node->uncle];
//             postponed >>= 1;
//             postponedBits--;
//             if (stop)
//             {
//                 return;
//             }
//         }
//         node = nullptr;
//     };

//     while (key)
//     {
//         if (bits > 32)
//             throw std::runtime_error("BVH intersect max depth achieved");
//         if (node->isLeaf())
//         {
//             auto base = node->first;
//             for (int i = 0; i < node->count; i++)
//                 result |= intersectElement(hit, ray, _indices[base + i]);
//             up();
//             continue;
//         }
//         float L0, L1;
//         float R0, R1;
//         bool iL = _node_intersect(L0, L1, node->leftBox, ray.origin, D_1)
//             && L0 < hit.t;
//         bool iR = node->right != BVH::EMPTY
//             && _node_intersect(R0, R1, node->rightBox, ray.origin, D_1)
//             && R0 < hit.t;

//         // going up
//         if (!(iL || iR))
//         {
//             up();
//             continue;
//         }

//         // going down
//         postponed <<= 1;
//         postponedBits++;
//         if (iL)
//         {
//             if (iR)
//             {
//                 postponed |= 1; // postpone a node
//                 node = L0 < L1
//                     ? &_nodes[node->left]
//                     : &_nodes[node->right];
//             }
//             else // only left
//                 node = &_nodes[node->left];
//         }
//         else if (iR) // only right
//         {
//             node = &_nodes[node->right];
//         }
//     }

//     return result;
// }

namespace binarytree
{

constexpr uint32_t parent(uint32_t node) { return node >> 1U; }

constexpr uint32_t sibling(uint32_t node) { return node ^ 1U; }

constexpr uint32_t left(uint32_t node) { return node << 1U; }

constexpr uint32_t right(uint32_t node) { return sibling(left(node)); }

constexpr uint32_t uncle(uint32_t node) { return sibling(parent(node)); }

} // namespace binarytree

} // namespace cg::rt
