#include "BezierSurface.h"

#include <fstream>

namespace cg
{

BezierSurface::~BezierSurface() {}

Ref<BezierSurface> BezierSurface::load(std::istream& input)
{
    Reference obj = new BezierSurface();

    uint32_t nPatches;
    // Skip whitespace.
    input >> std::skipws >> nPatches;
    if (!input) throw std::runtime_error("Could not read the no. patches");

    auto& indices = obj->_indices;
    indices.resize(16 * nPatches);

    // For each line, a patch.
    for (uint32_t p = 0; p < nPatches; p++)
    {
        // For each patch, 16 points with a comma between them.
        for (uint32_t i = 0; i < 16; i++)
        {
            uint32_t value;
            input >> value;

            indices[16*p + i] = value - 1; // Start index from zero.

            if (!input) throw std::runtime_error("Parsing patch failed");
            // Skip the comma, or any separator, or even the newline.
            input.ignore();
        }
    }

    // Read points.
    uint32_t nPoints;
    input >> nPoints;
    if (!input) throw std::runtime_error("Could not read the no. vertices");

    auto& points = obj->_points;
    points.resize(nPoints);

    for (uint32_t i = 0; i < nPoints; i++)
    {
        auto& point = points[i];
        point.w = 1.0f;
        for (uint32_t k = 0; k < 3; k++)
        {
            input >> point[k];
            if (!input) throw std::runtime_error("Parsing point failed");
            // Skip the comma, or any separator, or even the newline.
            input.ignore();
        }
    }

    return obj;
}

Ref<BezierSurface> BezierSurface::load(const char* filename)
{
    std::ifstream file (filename, std::ios::in);

    if (!file)
        return nullptr;

    return load(file);
}

} // namespace cg
