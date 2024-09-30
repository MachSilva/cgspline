#include "GLSurface.h"

#include <fstream>

namespace cg
{

GLSurface::GLSurface()
{
    glGenVertexArrays(1, &_vao);
    glBindVertexArray(_vao);

    _points = new GLStorage<vec4f>();
    glBindBuffer(GL_ARRAY_BUFFER, *_points);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);

    _indices = new GLStorage<uint32_t>();
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *_indices);
}

GLSurface::~GLSurface()
{
    glDeleteVertexArrays(1, &_vao);
}

GLSurface::GLSurface(const PatchTable& another)
    : GLSurface()
{
    _points->resize(another.points.size(), GL_STATIC_DRAW);
    _points->setData(another.points.data());
    
    _indices->resize(another.indices.size(), GL_STATIC_DRAW);
    _indices->setData(another.indices.data());

    if (!another.matrices.empty())
    {
        _matrices = new GLStorage<float>
            (another.matrices.size(), another.matrices.data(), GL_STATIC_DRAW);
        // _matrices->resize(another.matrices.size(), GL_STATIC_DRAW);
        // _matrices->setData(another.matrices.data());
    }

    _groups = another.groups;
}

} // namespace cg
