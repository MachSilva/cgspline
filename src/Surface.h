#pragma once

#include <math/Vector4.h>
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

constexpr auto PatchTypeCount = 3U;

enum PatchType : int8_t
{
    PatchType_Bezier,
    PatchType_BSpline,
    PatchType_Gregory
};

constexpr int patchTypeOriginalSize(PatchType t) noexcept
{
    switch (t)
    {
    case PatchType_Bezier: return 16;
    case PatchType_BSpline: return 16;
    case PatchType_Gregory: return 20;
    default: return 0;
    }
};


struct PatchTable
{
    struct PatchData;
    struct PatchGroup;
    struct PatchRef;

    std::vector<vec4> points;
    std::vector<uint32_t> indices;
    std::vector<float> matrices;
    std::vector<PatchGroup> groups;

    template<std::invocable<PatchRef> F>
    void forEachPatch(F&& f) const;

    PatchRef find(int i) const;

    void set(std::span<PatchData> data);
};

struct PatchTable::PatchData
{
    std::vector<uint32_t> indices;
    std::vector<float> matrix;
    PatchType type {};

    int matrixColumnCount() const noexcept
    {
        return indices.size();
    }

    int matrixRowCount() const noexcept
    {
        return patchTypeOriginalSize(type);
    }
};

// Group patches of the same type and size
struct PatchTable::PatchGroup
{
    int offset = 0;         //< start offset of control point indices
    int matrixOffset = -1;  //< start offset of matrix elements (negative if absent)
    uint16_t count = 0;     //< patch count
    uint16_t size = 0;      //< size (no. indices) per patch
    PatchType type {};

    constexpr bool invalid() const noexcept { return size < 1; }
};

struct PatchTable::PatchRef
{
    int offset = 0;         //< start offset of control point indices
    int matrixOffset = -1;  //< start offset of patch matrix (negative if absent)
    uint16_t size = 0;      //< source control point count
    PatchType type {};

    // constexpr int matrixSize() const noexcept
    // {
    //     return patchTypeOriginalSize(type) * size;
    // }
};

template<std::invocable<PatchTable::PatchRef> F>
void PatchTable::forEachPatch(F&& f) const
{
    for (auto& g : groups)
    {
        int offsetEnd = g.offset + g.size * g.count;
        if (g.matrixOffset < 0)
        {
            for (int p = g.offset; p < offsetEnd; p += g.size)
            {
                f(PatchRef{ .offset = p, .matrixOffset = -1,
                    .size = g.size, .type = g.type });
            }
        }
        else
        {
            int matrixSize = g.size * patchTypeOriginalSize(g.type);
            int matrixOffset = g.matrixOffset;
            for (int p = g.offset; p < offsetEnd; p += g.size)
            {
                f(PatchRef{ .offset = p, .matrixOffset = matrixOffset,
                    .size = g.size, .type = g.type });
                matrixOffset += matrixSize;
            }
        }
    }
}

class Surface : public SharedObject
{
public:
    Surface() = default;
    // ~Surface() override;

    Surface(PatchTable&& p) : _patches{std::move(p)} {}

    uint32_t count() const
    {
        uint32_t n = 0;
        for (auto& g : _patches.groups)
            n += g.count;
        return n;
    };

    std::span<vec4> points() { return _patches.points; }
    std::span<uint32_t> indices() { return _patches.indices; }

    std::span<const vec4> points() const { return _patches.points; }
    std::span<const uint32_t> indices() const { return _patches.indices; }

    auto& patches() { return _patches; }
    const auto& patches() const { return _patches; }
    
    static Ref<Surface> load(const char* filename);

    // Load bicubic bezier elements (".be")
    static Ref<Surface> load_be(std::istream& input);

    // Load surface elements (".se")
    static Ref<Surface> load_se(std::istream& input);

private:
    PatchTable _patches;
};

} // namespace cg
