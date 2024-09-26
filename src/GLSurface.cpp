#include "GLBezierSurface.h"

#include <fstream>

namespace cg
{

GLBezierSurface::GLBezierSurface()
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

GLBezierSurface::~GLBezierSurface()
{
    glDeleteVertexArrays(1, &_vao);
}

GLBezierSurface::GLBezierSurface(const BezierSurface& another)
    : GLBezierSurface()
{
    std::span aPoints = another.points();
    _points->resize(aPoints.size(), GL_STATIC_DRAW);
    _points->setData(aPoints.data());
    
    std::span aIndices = another.indices();
    _indices->resize(aIndices.size(), GL_STATIC_DRAW);
    _indices->setData(aIndices.data());
}

} // namespace cg
