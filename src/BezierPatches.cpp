#include "BezierPatches.h"

#include <fstream>

namespace cg
{

BezierPatches::BezierPatches()
{
    glGenVertexArrays(1, &_vao);
    glBindVertexArray(_vao);

    _points = new GLStorage<vec4f>();
    glBindBuffer(GL_ARRAY_BUFFER, *_points);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);

    _indexes = new GLStorage<uint32_t>();
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *_indexes);
}

BezierPatches::~BezierPatches()
{
    glDeleteVertexArrays(1, &_vao);
}

BezierPatches::BezierPatches(std::istream& input)
    : BezierPatches()
    // : _degree{3U,3U}
{
    uint32_t nPatches;
    // Skip whitespace.
    input >> std::skipws >> nPatches;
    if (!input) throw std::runtime_error("Could not read the no. patches");

    _indexes->resize(16 * nPatches, GL_STATIC_DRAW);
    auto pIndexes = _indexes->map(GL_WRITE_ONLY);

    // For each line, a patch.
    for (uint32_t p = 0; p < nPatches; p++)
    {
        // For each patch, 16 points with a comma between them.
        for (uint32_t i = 0; i < 16; i++)
        {
            uint32_t value;
            input >> value;

            pIndexes[16*p + i] = value - 1; // Start index from zero.

            if (!input) throw std::runtime_error("Parsing patch failed");
            // Skip the comma, or any separator, or even the newline.
            input.ignore();
        }
    }

    // Read points.
    uint32_t nPoints;
    input >> nPoints;
    if (!input) throw std::runtime_error("Could not read the no. vertices");

    _points->resize(nPoints, GL_STATIC_DRAW);
    auto pPoints = _points->map(GL_WRITE_ONLY);

    for (uint32_t i = 0; i < nPoints; i++)
    {
        auto& point = pPoints[i];
        point.w = 1.0f;
        for (uint32_t k = 0; k < 3; k++)
        {
            input >> point[k];
            if (!input) throw std::runtime_error("Parsing point failed");
            // Skip the comma, or any separator, or even the newline.
            input.ignore();
        }
    }
}

Reference<BezierPatches> BezierPatches::load(const char* filename)
{
    std::ifstream file (filename, std::ios::in);

    if (!file)
        throw std::runtime_error(std::string("Failed to open file ")+filename);

    return new BezierPatches(file);
}

} // namespace cg
