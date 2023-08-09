#pragma once

#include <graphics/Actor.h>
#include <graphics/GLRenderer.h>
#include <graphics/PrimitiveMapper.h>
#include <graph/ComponentProxy.h>
#include <graph/PrimitiveProxy.h>
#include "BezierPatches.h"

namespace cg
{

namespace GLSL
{

/**
 * @brief GLSL functions that evaluate bicubic Bézier surfaces using deCasteljau
 * algorithm.
 * 
 * It defines many functions:
 * - vec4 eval(float u, float v);
 * - vec3 evalNormal(float u, float v);
 * - vec4 deCasteljau(float u, inout vec4 p[4]);
 * - vec4 deCasteljauDx(float u, inout vec4 p[4]);
 * - vec4 derivativeU(float u, float v);
 * - vec4 derivativeV(float u, float v);
 * 
 * It is up to you to provide the following symbols to your shader:
 * - vec4 pointAt(int i, int j);
 * 
 * @warning It does not work for Rational Bézier Surfaces.
 * @warning The derivative functions return vectors of INCORRECT length,
 * but of CORRECT direction.
 */
extern const char * const BICUBIC_DECASTELJAU_FUNCTIONS;

} // namespace GLSL

// The surface itself
class Surface : public Primitive
{
protected:
    bool localIntersect(const Ray3f&, Intersection&) const override;
    bool localIntersect(const Ray3f&) const override;

public:
    Surface(BezierPatches* patches) : _patches{patches} {}

    BezierPatches* patches() const { return _patches; }
    void setPatches(BezierPatches* p) { _patches = p; }

    vec3f normal(const Intersection&) const override;
    Bounds3f bounds() const override;

private:
    Reference<BezierPatches> _patches;
};

// Surface as some object that knows to render itself
class SurfaceMapper : public PrimitiveMapper
{
public:
    SurfaceMapper(BezierPatches* patches) : _surface{new Surface(patches)} {}

    void update() override;
    bool render(GLRenderer&) const override;

    Bounds3f bounds() const override;
    Primitive* primitive() const override;

    Surface& surface() { return *_surface; }

private:
    Reference<Surface> _surface;
};

// Surface as a component
class SurfaceProxy : public graph::ComponentProxy<SurfaceMapper>
{
public:
    SurfaceProxy(BezierPatches* p) : ComponentProxy{"Surface", *new SurfaceMapper(p)} {}

    void afterAdded() override;
    void beforeRemoved() override;
    void transformChanged() override;
    void setVisible(bool value) override;

private:
    Reference<Actor> _actor;
};

class SurfacePipeline : public GLRenderer::Pipeline
{
public:
    SurfacePipeline(GLuint vertex, GLuint fragment);
    ~SurfacePipeline() override;

    GLSL::Program& tessellationProgram() { return _program; }

protected:
    GLSL::Program _program;
};

} // namespace cg
