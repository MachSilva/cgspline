#pragma once

#include <core/SharedObject.h>
#include <math/Vector4.h>
#include <geometry/Index2.h>
#include "GLStorage.h"

namespace cg
{

class BezierPatches : public SharedObject
{
public:
    BezierPatches();
    ~BezierPatches() override;

    // BezierPatches(vertices, patches);
    BezierPatches(std::istream& input);

    auto bind() const { glBindVertexArray(_vao); }
    auto count() const { return _indexes->size() / ((_degree.u+1) * (_degree.v+1)); };

    GLStorage<vec4f>* points() { return _points; }
    GLStorage<uint32_t>* indexes() { return _indexes; }
    
    static
    Reference<BezierPatches> load(const char* filename);

private:
    GLuint _vao;
    Reference<GLStorage<vec4f>> _points {nullptr};
    Reference<GLStorage<uint32_t>> _indexes {nullptr};
    const Index2<uint16_t> _degree {3U,3U};
};

} // namespace cg
