#include "Surface.h"

#include <graphics/GLRenderer.h>
#include <graph/Scene.h>

#define STRINGIFY(s) #s

namespace cg
{

namespace GLSL
{

const char * const BICUBIC_DECASTELJAU_FUNCTIONS = STRINGIFY(
    vec4 _p[4];
    vec4 _q[4];
    const int _degU = 3;
    const int _degV = 3;

    vec4 pointAt(int i, int j);

    vec4 deCasteljau(float u, inout vec4 p[4])
    {
        p[0] += u * (p[1] - p[0]);
        p[1] += u * (p[2] - p[1]);
        p[2] += u * (p[3] - p[2]);

        p[0] += u * (p[1] - p[0]);
        p[1] += u * (p[2] - p[1]);

        return p[0] + u * (p[1] - p[0]);
    }

    vec4 deCasteljauDx(float u, inout vec4 p[4])
    {
        p[0] = p[1] - p[0];
        p[1] = p[2] - p[1];
        p[2] = p[3] - p[2];

        p[0] += u * (p[1] - p[0]);
        p[1] += u * (p[2] - p[1]);

        return p[0] + u * (p[1] - p[0]);
    }

    vec4 eval(float u, float v)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
                _p[i] = pointAt(i, j);
            _q[j] = deCasteljau(u, _p);
        }
        return deCasteljau(v, _q);
    }

    vec4 derivativeU(float u, float v)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
                _p[i] = pointAt(i, j);
            _q[j] = deCasteljauDx(u, _p);
        }
        return deCasteljau(v, _q);
        // INCORRECT length, but CORRECT direction.
    }

    vec4 derivativeV(float u, float v)
    {
        // for (int j = 0; j < 4; j++)
        // {
        //     for (int i = 0; i < 4; i++)
        //         _p[i] = pointAt(i, j);
        //     _q[j] = deCasteljau(u, _p);
        // }
        // return deCasteljauDx(v, _q);

        // -----

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
                _q[j] = pointAt(i, j);
            _p[i] = deCasteljauDx(v, _q);
        }
        return deCasteljau(u, _p);
    }

    const float fixValue = 0.0001;
    // WARNING: It does not work for Rational Bézier Surfaces.
    vec3 evalNormal(float u, float v)
    {
        vec3 derU = derivativeU(u, v).xyz;
        vec3 derV = derivativeV(u, v).xyz;
        // Fix for degenerated patches.
        if (derU == vec3(0))
        {
            derU = derivativeU(u, abs(v - fixValue)).xyz;
        }
        if (derV == vec3(0))
        {
            derV = derivativeV(abs(u - fixValue), v).xyz;
        }
        return normalize(cross(derU, derV));
    }
);

} // namespace GLSL

static const char* _glslVersion = "#version 430 core\n";

static const char* _vs = STRINGIFY(
    out gl_PerVertex
    {
        vec4 gl_Position;
    };

    layout(location = 0) in vec4 position;

    void main()
    {
        gl_Position = position;
    }
);

// Depends on MatrixBlock
static const char* _tcs = STRINGIFY(
    in gl_PerVertex
    {
        vec4 gl_Position;
    } gl_in[];

    out gl_PerVertex
    {
        vec4 gl_Position;
    } gl_out[];

    layout(vertices = 16) out;

    const vec3 limit = vec3(1.0);
    float level()
    {
        vec3 bounds0 = vec3(2.0);
        vec3 bounds1 = vec3(-2.0);

        for (int i = 0; i < 16; i++)
        {
            vec4 p = mvpMatrix * gl_in[i].gl_Position;
            p.xyz /= p.w;

            bounds0 = min(bounds0, p.xyz);
            bounds1 = max(bounds1, p.xyz);
        }

        if (any(greaterThan(bounds0, limit)) || any(lessThan(bounds1, -limit)))
            return 0.0; // Discard patch before tessellation.

        vec2 rect = (bounds1.xy - bounds0.xy) * 20.0;
        return clamp(max(rect.x, rect.y), 2.0, 64.0);
    }

    void main()
    {
        gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
        if (gl_InvocationID == 0)
        {
            // float value = level();

            gl_TessLevelOuter[0] = gl_TessLevelOuter[1] =
            gl_TessLevelOuter[2] = gl_TessLevelOuter[3] = 
            gl_TessLevelInner[0] = gl_TessLevelInner[1] = 8;
        }
    }
);

// Depends on MatrixBlock:
// - mat4 mvpMatrix
static const char* _tes = STRINGIFY(
    in gl_PerVertex
    {
        vec4 gl_Position;
    } gl_in[];

    out gl_PerVertex
    {
        vec4 gl_Position;
    };

    layout(quads, equal_spacing, ccw) in;

    layout(location = 0) out vec3 v_position;
    layout(location = 1) out vec3 v_normal;
    layout(location = 2) out vec4 v_uv;

    vec4 pointAt(int i, int j)
    {
        return gl_in[4*j + i].gl_Position;
    }

    vec4 eval(float u, float v);
    vec3 evalNormal(float u, float v);

    void main()
    {
        vec4 P = eval(gl_TessCoord.x, gl_TessCoord.y);
        vec3 N = normalize(normalMatrix * evalNormal(gl_TessCoord.x, gl_TessCoord.y));
        gl_Position = mvpMatrix * P;
        vec4 Q = mvMatrix * P;
        v_position = Q.xyz / Q.w;
        v_normal = N;
        v_uv = vec4(gl_TessCoord.xy, 0, 0);
    }
);

SurfacePipeline::SurfacePipeline(GLuint vertex, GLuint fragment):
    _program{"Bézier Tessellation"}
{
    _program.setShader(GL_VERTEX_SHADER, {_glslVersion, _vs});
    _program.setShader(GL_TESS_CONTROL_SHADER, {
        _glslVersion,
        GLSL::MATRIX_BLOCK_DECLARATION,
        _tcs
    });
    _program.setShader(GL_TESS_EVALUATION_SHADER, {
        _glslVersion,
        GLSL::MATRIX_BLOCK_DECLARATION,
        GLSL::BICUBIC_DECASTELJAU_FUNCTIONS,
        _tes
    });
    _program.setSeparable(true);
    // _program.link();
    _program.use();

    glUseProgramStages(_pipeline, GL_VERTEX_SHADER_BIT, vertex ? vertex : _program);
    glUseProgramStages(_pipeline, GL_FRAGMENT_SHADER_BIT, fragment);
    glUseProgramStages(
        _pipeline,
        GL_TESS_CONTROL_SHADER_BIT | GL_TESS_EVALUATION_SHADER_BIT,
        _program
    );

    glActiveShaderProgram(_pipeline, _program);
}

SurfacePipeline::~SurfacePipeline() {}

bool Surface::localIntersect(const Ray3f&, Intersection&) const
{
    return false;
}

bool Surface::localIntersect(const Ray3f&) const
{
    return false;
}

vec3f Surface::normal(const Intersection&) const
{
    return {};
}

Bounds3f Surface::bounds() const
{
    return {};
}

void SurfaceMapper::update()
{
    // ???
}

bool SurfaceMapper::render(GLRenderer& renderer) const
{
    auto pipeline = renderer.pipeline(GLRenderer::PipelineCode::Surface);
    if (!pipeline) return false;

    auto c = renderer.camera();
    auto s = _surface->patches();

    auto b = static_cast<GLSL::MatrixBlock*>(glMapNamedBuffer(renderer.matrixBlock(), GL_WRITE_ONLY));

    mat4f mv = c->worldToCameraMatrix() * _surface->localToWorldMatrix();
    b->mvMatrix = mv;
    b->mvpMatrix = c->projectionMatrix() * mv;
    b->normalMatrix = mat3f{c->worldToCameraMatrix()} * _surface->normalMatrix();

    glUnmapNamedBuffer(renderer.matrixBlock());

    pipeline->use();
    s->bind();

    // glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPatchParameteri(GL_PATCH_VERTICES, 16);
    glDrawElements(GL_PATCHES, s->indexes()->size(), GL_UNSIGNED_INT, 0);

    return true;
}

Bounds3f SurfaceMapper::bounds() const
{
    return {};
}

Primitive* SurfaceMapper::primitive() const
{
    return _surface;
}

// Same as PrimitiveProxy

void SurfaceProxy::afterAdded()
{
    assert(sceneObject() != nullptr);
    _actor = new Actor{*_object};
    sceneObject()->scene()->addActor(_actor);
}

void SurfaceProxy::beforeRemoved()
{
    if (_actor != nullptr)
    {
        assert(sceneObject() != nullptr);
        sceneObject()->scene()->removeActor(_actor);
        _actor = nullptr;
    }
}

void SurfaceProxy::transformChanged()
{
    if (auto t = transform(); t->changed())
        if (auto p = _object->primitive())
            p->setTransform(t->localToWorldMatrix(), t->worldToLocalMatrix());
}

void SurfaceProxy::setVisible(bool value)
{
    if (_actor != nullptr)
        _actor->visible = value;
}

} // namespace cg
