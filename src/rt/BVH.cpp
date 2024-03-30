#include "BVH.h"

#include <stack>

namespace cg::rt
{

void PerfectHashTable::build(const span<uint32_t> keys)
{
    const auto n = keys.size();
    auto s = uint32_t(n >> 1) - 1;

    assert(n > 0xFFFFFFFF &&
        "PerfectHashTable: table size too big for an unsigned 32-bit integer");
    assert(s > n && // bit flip
        "PerfectHashTable: key set is too small to need a hash table; "
        "displacement array size will be zero and its behaviour undefined");

    auto len = std::bit_floor(s);
    _displacement.resize(len);
    _displ_mask = len - 1U;
    _table_size = n;

    std::uniform_int_distribution<uint32_t> d (0, n);

    _h = HashFunction(d(_mt), d(_mt), d(_mt));

    // Start hashing keys
    // for (auto i = 0U; i < n; i++)
    // {

    // }
}

void BVH::build(span<ElementData> elements, uint32_t elementsPerNode)
{
    auto n = elements.size();
    _keys.clear();
    _nodes.clear();
    _indices.clear();
    _keys.reserve(2 * n);
    _nodes.reserve(2 * n);
    _indices.reserve(n);

    _elementsPerNode = elementsPerNode;
    _root = split(elements, ROOT_KEY);
    link(_root, EMPTY);
}

uint32_t BVH::split(span<ElementData> elements, uint32_t key)
{
    auto count = elements.size();
    if (count <= _elementsPerNode)
        return wrap(elements, key);

    Bounds3f box {};
    auto inv = 1.0f / count;

    const auto base = _indices.size();
    _indices.resize(base + count);
    for (auto i = 0U; i < count; i++)
    {
        auto& e = elements[i];
        _indices[base + i] = e.index;
        box.inflate(e.bounds);
    }

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

    _nodes.emplace_back(Node{
        .left = split(elements.subspan(0, middle), binarytree::left(key)),
        .right = split(elements.subspan(middle), binarytree::right(key)),
        .uncle = EMPTY,
    });
    _keys.emplace_back(key);
    auto& node = _nodes.back();
    auto& left = _nodes[node.left];
    auto& right = _nodes[node.right];
    node.leftBox = left.leftBox;
    node.rightBox = right.leftBox;
    if (!left.isLeaf())
        node.leftBox.inflate(left.rightBox);
    if (!right.isLeaf())
        node.rightBox.inflate(right.rightBox);
    return _nodes.size() - 1;
}
uint32_t BVH::wrap(span<ElementData> elements, uint32_t key)
{
    auto count = elements.size();
    if (count == 0)
        throw std::logic_error("invalid");
        // return EMPTY;

    Bounds3f box {};
    const auto idx = _indices.size();
    _indices.resize(idx + count);
    for (auto i = 0U; i < count; i++)
    {
        auto& e = elements[i];
        _indices[idx + i] = e.index;
        box.inflate(e.bounds);
    }

    _nodes.emplace_back(Node{
        .leftBox = box,
        .rightBox = {},
        .left = EMPTY,
        // .right = EMPTY,
        // .uncle = EMPTY,
    });
    _keys.emplace_back(key);
    auto& node = _nodes.back();
    node.first = idx;
    node.count = count;
    return _nodes.size() - 1;
}

void BVH::link(uint32_t node, uint32_t uncle)
{
    auto& e = _nodes[node];
    e.uncle = uncle;

    if (e.isLeaf())
        return;

    // Uncle must exist for all non-leaf non-root nodes
    // All non-leaf nodes must have two children
    // Non-leaf nodes with a single child must be converted to a leaf node
    link(e.left, e.right);
    link(e.right, e.left);
}

static inline
bool _node_intersect(float& tMin, float& tMax, const Bounds3f& bounds,
    const vec3f& origin, const vec3f& directionInverse)
{
    using flt = std::numeric_limits<float>;
    static_assert(flt::is_iec559, "Quiet NaNs and infinities required");

    auto& b0 = bounds.min();
    auto& b1 = bounds.max();
    tMin = -flt::infinity();
    tMax = +flt::infinity();

    // y = A + Bt
    // t = (y - A) / B, B != 0
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

bool BVH::intersect(
    Intersection& hit,
    const Ray& ray,
    std::function<bool(Intersection&, const Ray&, uint32_t)> intersectElement
    ) const
{
    if (_root == BVH::EMPTY)
        return false;

    const vec3f D_1 = ray.direction.inverse();
    bool result = false;

    std::deque<const Node*> S;
    S.clear();
    S.push_back(&_nodes[_root]);

    while (!S.empty())
    {
        const Node* node = S.back();
        S.pop_back();

        if (node->isLeaf())
        {
            auto base = node->first;
            for (int i = 0; i < node->count; i++)
            {
                bool b = intersectElement(hit, ray, _indices[base + i]);
                result |= b;
            }
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


bool BVH::intersects(
    const Ray& ray,
    std::function<bool(const Ray&, uint32_t)> intersectsElement
    ) const
{
    if (_root == BVH::EMPTY)
        return false;

    const vec3f D_1 = ray.direction.inverse();
    uint32_t offset = _root;

    std::deque<const Node*> S;
    S.clear();
    S.push_back(&_nodes[_root]);

    while (!S.empty())
    {
        auto node = S.back();
        S.pop_back();

        if (node->isLeaf())
        {
            auto base = node->first;
            for (int i = 0; i < node->count; i++)
            {
                if (intersectsElement(ray, _indices[base + i]))
                {
                    return true;
                }
            }
            continue;
        }

        float L0, L1;
        float R0, R1;
        bool iL = _node_intersect(L0, L1, node->leftBox, ray.origin, D_1)
            && L0 < ray.tMax;
        bool iR = node->right != BVH::EMPTY
            && _node_intersect(R0, R1, node->rightBox, ray.origin, D_1)
            && R0 < ray.tMax;

        if (iL)
            S.push_back(&_nodes[node->left]);
        if (iR)
            S.push_back(&_nodes[node->right]);
    }

    return false;
}

} // namespace cg::rt
