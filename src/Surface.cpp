#include "Surface.h"

#include <fstream>

namespace cg
{

PatchData PatchTable::find(int i) const
{
    for (auto& g : groups)
    {
        if (i < g.count && i >= 0)
        {
            int m = -1;
            if (g.matrixOffset >= 0)
            {
                int matrixSize = g.size * patchTypeOriginalSize(g.type);
                m = g.matrixOffset + i * matrixSize;
            }
            return PatchData
            {
                .type = g.type,
                .size = g.size,
                .offset = g.offset + i * g.size,
                .matrixOffset = m,
            };
        }
        // else
        i -= g.count;
    }
    return {};
}

void PatchTable::group()
{

}

void PatchTable::set(std::span<const PatchData> data)
{

}

Surface::~Surface() {}

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

            t.indices[16*p + i] = value - 1; // Start index from zero.

            if (!input) throw std::runtime_error("Parsing patch failed");
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

    t.groups.push_back(PatchGroup{
        .type = PatchType_Bezier,
        .size = 16,
        .count = (int) nPatches,
        .offset = 0,
    });

    return new Surface(std::move(t));
}

Ref<Surface> Surface::load_se(std::istream& input)
{
    return nullptr;
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
