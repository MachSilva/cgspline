#pragma once

#include <graphics/Actor.h>
#include <graphics/GLRenderer.h>
#include <graphics/PrimitiveMapper.h>
#include <graph/ComponentProxy.h>
#include <graph/PrimitiveProxy.h>
#include "BezierPatches.h"
#include "PatchBVH.h"
#include "PBRMaterial.h"

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
class SurfacePrimitive : public Primitive
{
protected:
    bool localIntersect(const Ray3f&, Intersection&) const override;
    bool localIntersect(const Ray3f&) const override;

public:
    SurfacePrimitive() = default;

    SurfacePrimitive(BezierPatches* patches)
    {
        setPatches(patches);
    }

    BezierPatches* patches() const { return _patches; }
    void setPatches(BezierPatches* p)
    {
        _patches = p;
        _bvh = new PatchBVH(*p);
    }

    vec4f point(const Intersection&) const;
    vec3f normal(const Intersection&) const override;
    Bounds3f bounds() const override;

    Ref<PBRMaterial> pbrMaterial {nullptr};

private:
    Ref<BezierPatches> _patches;
    Ref<PatchBVH> _bvh;
};

// Surface as some object that knows to render itself
class SurfaceMapper : public PrimitiveMapper
{
public:
    SurfaceMapper(SurfacePrimitive* surface) : _surface{surface} {}

    void update() override;
    bool render(GLRenderer&) const override;
    bool renderContour(GLRenderer&) const;

    Bounds3f bounds() const override;
    Primitive* primitive() const override;

    SurfacePrimitive& surface() { return *_surface; }

protected:
    Ref<SurfacePrimitive> _surface;

    void updateMatrixBlock(GLRenderer& renderer) const;
};

// Surface as a component
class SurfaceProxy : public graph::PrimitiveProxy
{
public:
    SurfaceProxy(SurfacePrimitive* s)
        : graph::PrimitiveProxy{*new SurfaceMapper(s)} {}

    Actor* actor() const { return _actor; }
    SurfaceMapper* mapper() const
    {
        return (SurfaceMapper*) _object.get();
    }

protected:
    Ref<Actor> _actor;
};

class SurfacePipeline : public GLRenderer::Pipeline
{
public:
    enum Mode
    {
        Standard3D,
        ContourCurves,
    };

    SurfacePipeline(GLuint vertex, GLuint fragment, Mode mode = Standard3D);
    ~SurfacePipeline() override;

    GLSL::Program& tessellationProgram() { return _program; }

protected:
    GLSL::Program _program;
};

} // namespace cg
