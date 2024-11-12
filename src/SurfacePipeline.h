#pragma once

#include <graphics/Actor.h>
#include <graphics/GLRenderer.h>
#include <graphics/PrimitiveMapper.h>
#include <graph/ComponentProxy.h>
#include <graph/PrimitiveProxy.h>
#include "GLSurface.h"
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
 * - void evalAll(float u, float v, out vec4 P, out vec4 D);
 * 
 * It is up to you to provide the following symbols to your shader:
 * - vec4 point(int i);
 * 
 * @warning It does not work for Rational Bézier Surfaces.
 * @warning The derivative functions DOES NOT normalized returned vectors.
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

    SurfacePrimitive(Surface* s) { setSurface(s); }
    SurfacePrimitive(GLSurface* s) { setSurface(s); }

    GLSurface* surface() const { return _surface; }
    void setSurface(GLSurface* p)
    {
        _surface = p;
        _bvh = new PatchBVH(p);
    }

    void setSurface(Surface* p)
    {
        _surface = new GLSurface(p->patches());
        _bvh = new PatchBVH(p);
    }

    vec4f point(const Intersection&) const;
    vec3f normal(const Intersection&) const override;
    Bounds3f bounds() const override;

    Ref<PBRMaterial> pbrMaterial {nullptr};

private:
    Ref<GLSurface> _surface;
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

    SurfacePrimitive* surface() { return _surface; }

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
    enum Variant
    {
        eBezierElement,
        eBSplineElement,
        eGregoryElement,
        eBezierContour,
        eBSplineContour,
        eGregoryContour
    };

    SurfacePipeline(GLuint vertex, GLuint fragment);
    ~SurfacePipeline() override;

    GLuint get(Variant i) const { return i == 0 ? _pipeline : _extraPipelines[i-1]; }

    auto getPatch16TCS() const { return _Patch16TCS; }
    auto getPatch20TCS() const { return _Patch20TCS; }
    auto getBezierTES() const { return _BezierTES; }
    auto getBSplineTES() const { return _BSplineTES; }
    auto getGregoryTES() const { return _GregoryTES; }

protected:
    Ref<GLSL::ShaderProgram> _passthroughVS,
        _Patch16TCS, _Patch20TCS,
        _BezierTES, _BSplineTES, _GregoryTES,
        _Patch16ContourTCS, _Patch20ContourTCS,
        _BezierContourTES, _BSplineContourTES, _GregoryContourTES;

    GLuint _extraPipelines[5];
};

} // namespace cg
