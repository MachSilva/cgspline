#include "Surface.h"

#include <algorithm>
#include <fstream>

namespace cg
{

PatchTable::PatchRef PatchTable::find(int i) const
{
    for (auto& g : groups)
    {
        if (i < 0)
            break;
        if (i < g.count)
        {
            int m = -1;
            if (g.matrixOffset >= 0)
            {
                int matrixSize = g.size * patchTypeOriginalSize(g.type);
                m = g.matrixOffset + i * matrixSize;
            }
            return PatchRef
            {
                .offset = g.offset + i * g.size,
                .matrixOffset = m,
                .size = g.size,
                .type = g.type,
            };
        }
        // else
        i -= g.count;
    }
    return {};
}

void PatchTable::set(std::span<PatchData> elements)
{
    struct Bucket
    {
        struct Data
        {
            std::vector<uint32_t> indices;
            std::vector<float> matrix;
        };

        std::vector<Data> elements;
        uint16_t nodeCount {};
        bool hasMatrix {};
        PatchType type {};
    };

    std::vector<Bucket> buckets;
    buckets.reserve(16);

    uint32_t indexDataSize = 0;
    uint32_t matrixDataSize = 0;

    for (auto& p : elements)
    { 
        Bucket* bucket {};
        uint16_t nodeCount = p.indices.size();
        uint16_t pointCount = patchTypeOriginalSize(p.type);
        for (auto& b : buckets)
        {
            if (b.type == p.type &&
                b.nodeCount == nodeCount &&
                b.hasMatrix == (p.matrix.size() != 0))
            {
                bucket = &b;
                break;
            }
        }
        if (!bucket)
        {
            buckets.push_back({
                .nodeCount = nodeCount,
                .hasMatrix = (p.matrix.size() != 0),
                .type = p.type,
            });
            bucket = &buckets.back();
        }

        uint32_t matrixSize = pointCount * nodeCount;
        indexDataSize += nodeCount;

        // add element to group
        auto& element = bucket->elements.emplace_back();
        element.indices = std::move(p.indices);
        if (bucket->hasMatrix)
        {
            if (p.matrix.size() != matrixSize)
                throw std::runtime_error("invalid element matrix size");

            matrixDataSize += matrixSize;
            element.matrix = std::move(p.matrix);
        }
    }

    // group patches and write
    this->indices.resize(indexDataSize);
    this->matrices.resize(matrixDataSize);
    this->groups.reserve(buckets.size());

    int offset = 0;
    int matrixOffset = 0;
    for (auto& b : buckets)
    {
        this->groups.push_back({
            .offset = offset,
            .matrixOffset = b.hasMatrix ? matrixOffset : -1,
            .count = (uint16_t)b.elements.size(),
            .size = b.nodeCount,
            .type = (PatchType)b.type,
        });
        if (b.hasMatrix)
        {
            const auto matrixSize =
                b.nodeCount * patchTypeOriginalSize((PatchType)b.type);
            for (auto& e : b.elements)
            {
                assert(e.indices.size() == b.nodeCount);
                assert(e.matrix.size() == matrixSize);

                std::copy(e.indices.begin(), e.indices.end(),
                    this->indices.data() + offset);
                offset += e.indices.size();

                std::copy(e.matrix.begin(), e.matrix.end(),
                    this->matrices.data() + matrixOffset);
                matrixOffset += e.matrix.size();
            }
        }
        else
        {
            for (auto& e : b.elements)
            {
                assert(e.indices.size() == b.nodeCount);
                std::copy(e.indices.begin(), e.indices.end(),
                    this->indices.data() + offset);
                offset += e.indices.size();
            }
        }
    }
}

// Surface::~Surface() {}

Ref<Surface> Surface::load_be(std::istream& input)
{
    PatchTable t;

    uint32_t nPatches;
    // Skip whitespace.
    input >> std::skipws >> nPatches;
    if (!input) throw std::runtime_error("Could not read the no. patches");

    t.indices.resize(16 * nPatches);

    // For each line, a patch.
    for (uint32_t p = 0; p < nPatches; p++)
    {
        // For each patch, 16 points with a comma between them.
        for (uint32_t i = 0; i < 16; i++)
        {
            uint32_t value;
            input >> value;
            if (!input) throw std::runtime_error("Parsing patch failed");

            t.indices[16*p + i] = value - 1; // Start index from zero.

            // Skip the comma, or any separator, or even the newline.
            input.ignore();
        }
    }

    // Read points.
    uint32_t nPoints;
    input >> nPoints;
    if (!input) throw std::runtime_error("Could not read the no. vertices");

    t.points.resize(nPoints);

    for (uint32_t i = 0; i < nPoints; i++)
    {
        auto& point = t.points[i];
        point.w = 1.0f;
        for (uint32_t k = 0; k < 3; k++)
        {
            input >> point[k];
            if (!input) throw std::runtime_error("Parsing point failed");
            // Skip the comma, or any separator, or even the newline.
            input.ignore();
        }
    }

    // Check indices
    for (uint32_t i : t.indices)
    {
        if (t.points.size() < i)
            throw std::runtime_error
                ("Invalid/corrupted file. Indices out of range");
    }

    t.groups.push_back({
        .offset = 0,
        .count = (uint16_t) nPatches,
        .size = 16,
        .type = PatchType_Bezier,
    });

    return new Surface(std::move(t));
}

/**
 * @note {a}[3] is equivalent to {a} {a} {a}
 * 
 * Pseudo-Grammar -> {N} Node[N] {E} Element[E]
 * 
 * Node -> {id} {position:vec4}
 * Element ->
 *      {id} {type} {number of nodes} {degree} {boundary flag}
 *      {node refs}[number of nodes]
 *      {unknown}[number of nodes]
 *      MaybeBoundary<boundary flag>
 * 
 * MaybeBoundary<0> -> MatrixLine<type>[number of nodes]
 * MaybeBoundary<1> -> {:void}
 * 
 * MatrixLine<1> -> {:float}[16]
 * MatrixLine<2> -> {:float}[20]
 * 
 * type: BSpline=1, Gregory=2
 */
Ref<Surface> Surface::load_se(std::istream& input)
{
    PatchTable t;
    uint32_t nodeCountTotal;

    // discard comments and whitespace
    auto discard = [&]()
    {
        while (true)
        {
            int c = input.peek();
            if (c == '#')
            {
                input.get();
                while ((c = input.get()) >= 0 && c != '\n')
                    (void)0;
            }
            else if (isspace(c))
                input.get();
            else return;
        }
    };

    discard();
    input >> std::skipws >> nodeCountTotal;
    if (!input)
        throw std::runtime_error("could not read no. points / nodes");

    t.points.resize(nodeCountTotal);
    for (auto i = 0U; i < nodeCountTotal; i++)
    {
        auto& v = t.points[i];
        uint16_t id;
        discard();
        input >> id >> v.x >> v.y >> v.z >> v.w;
        if (i != id)
            throw std::runtime_error("unexpected point id");
    }

    uint32_t elementCount;
    discard();
    input >> elementCount;

    uint32_t indexDataSize = 0;
    uint32_t matrixDataSize = 0;

    std::vector<PatchTable::PatchData> elements;
    elements.reserve(elementCount);

    for (auto i = 0U; i < elementCount; i++)
    {
        uint16_t id, typeRead, nodeCount, degree, boundaryFlag;
        discard();
        input >> id >> typeRead >> nodeCount >> degree >> boundaryFlag;

        if (!input)
            throw std::runtime_error("read failed during element header");
        if (degree != 3)
            throw std::runtime_error("degree value unsupported");

        // add element to group
        auto& element = elements.emplace_back();

        uint16_t lines; // matrix lines
        if (typeRead == 1) // BSpline
        {
            lines = boundaryFlag ? 16 : 0;
            element.type = PatchType_BSpline;
        }
        else if (typeRead == 2) // Gregory
        {
            lines = 20;
            element.type = PatchType_Gregory;
        }
        else
            throw std::runtime_error("unknown patch type");

        int matrixSize = lines * nodeCount;
        indexDataSize += nodeCount;
        matrixDataSize += matrixSize;

        // node indices
        element.indices.resize(nodeCount);
        discard();
        for (int j = 0; j < nodeCount; j++)
        {
            input >> element.indices[j];
            if (nodeCountTotal < element.indices[j])
                throw std::runtime_error
                    ("Invalid/corrupted file. Indices out of range");
        }

        // an unknown value array
        int v;
        discard();
        for (int j = 0; j < nodeCount; j++)
            input >> v;

        // matrix
        element.matrix.resize(matrixSize);
        for (int j = 0; j < lines; j++)
        {
            discard();
            for (int k = 0; k < nodeCount; k++)
            {
                // transpose matrix
                input >> element.matrix[j + k*lines];
            }
        }
    }

    t.set(elements);
    return new Surface(std::move(t));
}

Ref<Surface> Surface::load(const char* filename)
{
    std::ifstream file (filename, std::ios::in);

    if (!file)
        return nullptr;

    auto n = std::strlen(filename);
    if (n < 3)
        return nullptr;

    auto base = n - 3;
    char s[4];
    for (int i = 0; i < 4; i++)
        s[i] = (char) tolower(filename[base+i]);

    if (s[0] == '.' && s[1] == 'b' && s[2] == 'e' && !s[3])
        return load_be(file);

    if (s[0] == '.' && s[1] == 's' && s[2] == 'e' && !s[3])
        return load_se(file);

    return load_be(file);
}

} // namespace cg
