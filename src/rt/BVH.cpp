#include "BVH.h"

#include <algorithm>
#include <stack>

namespace cg::rt
{

void BVH::build(std::span<ElementData> elements, uint32_t elementsPerNode)
{
    auto n = elements.size();
    _keys.clear();
    _nodes.clear();
    _indices.clear();
    _keys.reserve(2 * n);
    _nodes.reserve(2 * n);
    _indices.reserve(n);

    _elementsPerNode = elementsPerNode;
    split(elements, ROOT_KEY);
    link(_nodes.data(), EMPTY);
}

void BVH::buildHashTable(cudaStream_t stream)
{
    auto e = _hash.build(_keys, stream);
    if (e)
        throw std::runtime_error("hash table build failed");

    _table.resize(_hash.tableSize());
    std::ranges::uninitialized_fill(_table, EMPTY);

    for (int i = 0; i < _keys.size(); i++)
    {
        auto k = _keys[i];
        _table[_hash(k)] = i;
    }
}

uint32_t BVH::split(std::span<ElementData> elements, uint32_t key)
{
    auto count = elements.size();
    if (count <= _elementsPerNode)
        return wrap(elements, key);

    Bounds3f box {};
    auto inv = 1.0f / count;

    auto v = box.size();
    int dim = v.z > v.y
        ? (v.x > v.z ? 0 : 2)
        : (v.x > v.y ? 0 : 1);

    auto middle = count / 2;
    std::nth_element(
        elements.begin(),
        elements.begin() + middle,
        elements.end(),
        [=](const ElementData& a, const ElementData& b) -> bool
        {
            return a.centroid[dim] < b.centroid[dim];
        });

    _keys.push_back(key);
    auto idx = _nodes.size();
    auto& node = _nodes.emplace_back();
    node.left = split(elements.subspan(0, middle), binarytree::left(key)),
    node.right = split(elements.subspan(middle), binarytree::right(key)),
    node.uncle = EMPTY;

    auto& left = _nodes[node.left];
    auto& right = _nodes[node.right];
    node.leftBox = left.leftBox;
    node.rightBox = right.leftBox;
    if (!left.isLeaf())
        node.leftBox.inflate(left.rightBox);
    if (!right.isLeaf())
        node.rightBox.inflate(right.rightBox);
    return idx;
}

uint32_t BVH::wrap(std::span<ElementData> elements, uint32_t key)
{
    auto count = elements.size();
    if (count == 0)
        throw std::logic_error("invalid");
        // return EMPTY;

    Bounds3f box {};
    const auto idx = _indices.size();

#ifdef SPL_BVH_INDEX_SENTINEL
    _indices.resize(idx + count + 1);
#else
    _indices.resize(idx + count);
#endif
    for (auto i = 0U; i < count; i++)
    {
        auto& e = elements[i];
        _indices[idx + i] = e.index;
        box.inflate(e.bounds);
    }
#ifdef SPL_BVH_INDEX_SENTINEL
    _indices[idx + count] = EMPTY;
#endif

    _keys.push_back(key);
    auto& node = _nodes.emplace_back();
    node.leftBox = box;
    node.rightBox = {};
    node.left = EMPTY;
    node.first = idx;
#ifndef SPL_BVH_INDEX_SENTINEL
    node.count = count;
#endif
    return _nodes.size() - 1;
}

void BVH::link(Node* node, uint32_t sibling)
{
    if (node->isLeaf())
        return;

    // Uncle must exist for all non-leaf non-root nodes
    // All non-leaf nodes must have two children
    // Non-leaf nodes with a single child must be converted to a leaf node
    auto left = &_nodes[node->left];
    auto right = &_nodes[node->right];
    left->uncle = sibling;
    right->uncle = sibling;
    link(left, node->right);
    link(right, node->left);
}

bool BVH::intersect(
    Intersection& hit,
    const Ray& ray,
    std::function<bool(Intersection&, const Ray&, uint32_t)> intersectfn
    ) const
{
    const vec3f D_1 = ray.direction.inverse();
    bool result = false;

    std::deque<const Node*> S;
    S.clear();
    S.push_back(&_nodes[0]);

    while (!S.empty())
    {
        const Node* node = S.back();
        S.pop_back();

        if (node->isLeaf())
        {
#ifdef SPL_BVH_INDEX_SENTINEL
            auto i = node->first;
            while (_indices[i] != EMPTY)
            {
                bool b = intersectfn(hit, ray, _indices[i++]);
                result |= b;
            }
#else
            auto base = node->first;
            for (int i = 0; i < node->count; i++)
            {
                bool b = intersectfn(hit, ray, _indices[base + i]);
                result |= b;
            }
#endif
            continue;
        }

        float L0, L1;
        float R0, R1;
        bool iL = _node_intersect(L0, L1, node->leftBox, ray.origin, D_1)
            && L0 < hit.t;
        bool iR = _node_intersect(R0, R1, node->rightBox, ray.origin, D_1)
            && R0 < hit.t;

        if (iL)
        {
            if (iR)
            {
                if (L0 < R0)
                {
                    S.push_back(&_nodes[node->right]);
                    S.push_back(&_nodes[node->left]); // first out
                }
                else
                {
                    S.push_back(&_nodes[node->left]);
                    S.push_back(&_nodes[node->right]); // first out
                }
            }
            else // only left
            {
                S.push_back(&_nodes[node->left]);
            }
        }
        else if (iR) // only right
        {
            S.push_back(&_nodes[node->right]);
        }
    }

    return result;
}

bool BVH::intersect(
    const Ray& ray,
    std::function<bool(const Ray&, uint32_t)> intersectfn
    ) const
{
    const vec3f D_1 = ray.direction.inverse();
    uint32_t offset = 0;

    std::deque<const Node*> S;
    S.clear();
    S.push_back(&_nodes[0]);

    while (!S.empty())
    {
        auto node = S.back();
        S.pop_back();

        if (node->isLeaf())
        {
#ifdef SPL_BVH_INDEX_SENTINEL
            auto i = node->first;
            while (_indices[i] != EMPTY)
            {
                if (intersectfn(ray, _indices[i++]))
                {
                    return true;
                }
            }
#else
            auto base = node->first;
            for (int i = 0; i < node->count; i++)
            {
                if (intersectfn(ray, _indices[base + i]))
                {
                    return true;
                }
            }
#endif
            continue;
        }

        float L0, L1;
        float R0, R1;
        bool iL = _node_intersect(L0, L1, node->leftBox, ray.origin, D_1)
            && L0 < ray.max;
        bool iR = _node_intersect(R0, R1, node->rightBox, ray.origin, D_1)
            && R0 < ray.max;

        if (iL)
            S.push_back(&_nodes[node->left]);
        if (iR)
            S.push_back(&_nodes[node->right]);
    }

    return false;
}

} // namespace cg::rt
