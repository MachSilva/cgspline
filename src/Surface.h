#pragma once

#include <math/Vector4.h>
#include <geometry/Index2.h>
#include <cinttypes>
#include <span>
#include <vector>
#include "AlignedTypes.h"
#include "Ref.h"

namespace cg
{

using spline::vec2;
using spline::vec3;
using spline::vec4;
using spline::mat3;
using spline::mat4;

// maybe for some future extension...
class Surface : public SharedObject
{
    // nothing
};

class BezierSurface : public Surface
{
public:
    BezierSurface() = default;
    ~BezierSurface() override;

    BezierSurface(uint32_t vertexCount, uint32_t patchCount)
        : _points((size_t) vertexCount), _indices((size_t) 16 * patchCount) {}

    BezierSurface(std::vector<vec4>&& vPoints, std::vector<uint32_t>&& vIndices)
        : _points{std::move(vPoints)}, _indices{std::move(vIndices)} {}

    auto count() const { return _indices.size() / ((_degree.u+1) * (_degree.v+1)); };

    std::span<vec4> points() { return _points; }
    std::span<uint32_t> indices() { return _indices; }

    std::span<const vec4> points() const { return _points; }
    std::span<const uint32_t> indices() const { return _indices; }
    
    static Ref<BezierSurface> load(const char* filename);
    static Ref<BezierSurface> load(std::istream& input);

private:
    std::vector<vec4> _points;
    std::vector<uint32_t> _indices;
    static constexpr const struct
    {
        uint16_t u, v;
    } _degree { .u = 3U, .v = 3U };
};

} // namespace cg
