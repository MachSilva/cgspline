#pragma once

#include <math/Vector4.h>
#include "Surface.h"
#include "GLStorage.h"

namespace cg
{

class GLSurface : public SharedObject
{
public:
    GLSurface();
    ~GLSurface() override;

    GLSurface(const PatchTable&);

    auto bind() const { glBindVertexArray(_vao); }
    // auto count() const { return _count; }

    auto* points() { return _points.get(); }
    auto* indices() { return _indices.get(); }
    auto* matrices() { return _matrices.get(); }
    // auto* groups() { return _groups.get(); }

    const auto* points() const { return _points.get(); }
    const auto* indices() const { return _indices.get(); }
    const auto* matrices() const { return _matrices.get(); }
    // const auto* groups() const { return _groups.get(); }

    auto& groups() { return _groups; }
    auto& groups() const { return _groups; }

    static Ref<GLSurface> load(const char* filename)
    {
        auto s = Ref(Surface::load(filename));
        return new GLSurface(s->patches());
    }

    static Ref<GLSurface> load_be(std::istream& input)
    {
        auto s = Ref(Surface::load_be(input));
        return new GLSurface(s->patches());
    }

private:
    GLuint _vao;
    Ref<GLStorage<vec4f>> _points {};
    Ref<GLStorage<uint32_t>> _indices {};
    Ref<GLStorage<float>> _matrices {};
    std::vector<PatchGroup> _groups {};
};

} // namespace cg
