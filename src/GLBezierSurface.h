#pragma once

#include <math/Vector4.h>
#include <geometry/Index2.h>
#include "BezierSurface.h"
#include "GLStorage.h"

namespace cg
{

class GLBezierSurface : public Surface
{
public:
    GLBezierSurface();
    ~GLBezierSurface() override;

    GLBezierSurface(const BezierSurface&);

    auto bind() const { glBindVertexArray(_vao); }
    auto count() const { return _indices->size() / ((_degree.u+1) * (_degree.v+1)); };

    GLStorage<vec4f>* points() { return _points; }
    GLStorage<uint32_t>* indices() { return _indices; }

    const GLStorage<vec4f>* points() const { return _points; }
    const GLStorage<uint32_t>* indices() const { return _indices; }

    static Ref<GLBezierSurface> load(const char* filename)
    {
        return new GLBezierSurface(*BezierSurface::load(filename));
    }

    static Ref<GLBezierSurface> load(std::istream& input)
    {
        return new GLBezierSurface(*BezierSurface::load(input));
    }

private:
    GLuint _vao;
    Reference<GLStorage<vec4f>> _points {nullptr};
    Reference<GLStorage<uint32_t>> _indices {nullptr};
    static constexpr const struct
    {
        uint16_t u, v;
    } _degree { .u = 3U, .v = 3U };
    // const Index2<uint16_t> _degree {3U,3U};
};

} // namespace cg
