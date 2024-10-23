#include "SurfacePipeline.h"

#include <graphics/GLRenderer.h>
#include <graph/Scene.h>
#include "Spline.h"
#include "SplineMat.h"

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

const char * const PATCH_EVAL_FUNCTIONS = STRINGIFY(
    vec4 pointAt(int i, int j);

    vec4 evalBezierBasis(float u)
    {
        const float u2 = u*u;
        const float u3 = u2*u;
        return vec4(
            1 - 3*u + 3*u2 - u3,
            3*u - 6*u2 + 3*u3,
            3*u2 - 3*u3,
            u3
        );
    }

    // Evaluate bezier base function: B(u)
    // in  u: coordinate
    // out b: base function B(u) values per control point
    //     d: derivative function B'(u) values
    void evalBezier(float u, out vec4 b, out vec4 d)
    {
        const float u2 = u*u;
        const float u3 = u2*u;
        const float _3u2 = 3*u2;
        b = vec4(
            1 - 3*u + _3u2 - u3,
            3*u - 6*u2 + 3*u3,
            _3u2 - 3*u3,
            u3
        );
        d = vec4(
            -3 + 6*u - _3u2,
            3 - 12*u + 9*u2,
            6*u - 9*u2,
            _3u2
        );
    }

    const float fixValue = 0.0001;

    // Evaluate surface point: S(u,v)
    // out s:
    //     s[0]: S(u,v)
    //     s[1]: S'u(u,v)
    //     s[2]: S'v(u,v)
    void eval(float u, float v, out vec3 s[3])
    {
        vec4 bu;
        vec4 bdu;
        vec4 bv;
        vec4 bdv;
        evalBezier(u, bu, bdu);
        evalBezier(v, bv, bdv);
        s[0] = vec3(0);
        s[1] = vec3(0);
        s[2] = vec3(0);
        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                const vec3 P = pointAt(i,j).xyz;
                s[0] += (bu[i] * bv[j]) * P;
                s[1] += (bdu[i] * bv[j]) * P;
                s[2] += (bu[i] * bdv[j]) * P;
            }
        }
        // Fix for degenerated patches
        if (length(s[1]) <= 1e-6)
        {
            s[1] = vec3(0);
            bv = evalBezierBasis(abs(v - fixValue));
            for (int j = 0; j < 4; j++)
                for (int i = 0; i < 4; i++)
                    s[1] += (bdu[i] * bv[j]) * pointAt(i,j).xyz;
        }
        if (length(s[2]) <= 1e-6)
        {
            s[2] = vec3(0);
            bu = evalBezierBasis(abs(u - fixValue));
            for (int j = 0; j < 4; j++)
                for (int i = 0; i < 4; i++)
                    s[2] += (bu[i] * bdv[j]) * pointAt(i,j).xyz;
        }
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

        vec2 factor = 0.1 * vec2(viewportMatrix[0][0], viewportMatrix[1][1]);
        vec2 rect = (bounds1.xy - bounds0.xy) * factor;
        return clamp(max(rect.x, rect.y), 4.0, 64.0);
    }

    void main()
    {
        gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
        if (gl_InvocationID == 0)
        {
            float value = level();

            gl_TessLevelOuter[0] = gl_TessLevelOuter[1] =
            gl_TessLevelOuter[2] = gl_TessLevelOuter[3] = 
            gl_TessLevelInner[0] = gl_TessLevelInner[1] = value;
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

    void eval(float u, float v, out vec3 s[3]);
    // vec4 eval(float u, float v);
    // vec3 evalNormal(float u, float v);

    void main()
    {
        // vec4 P = eval(gl_TessCoord.x, gl_TessCoord.y);
        // vec3 N = normalize(normalMatrix * evalNormal(gl_TessCoord.x, gl_TessCoord.y));

        vec3 S[3]; // [P, Du, Dv]
        eval(gl_TessCoord.x, gl_TessCoord.y, S);
        vec3 N = normalize(normalMatrix * normalize(cross(S[1], S[2])));
        vec4 P = vec4(S[0], 1.0);
        gl_Position = mvpMatrix * P;
        vec4 Q = mvMatrix * P;
        v_position = Q.xyz / Q.w;
        v_normal = N;
        v_uv = vec4(gl_TessCoord.xy, 0, 1);
    }
);

// Depends on MatrixBlock
static const char* _tcsContour = STRINGIFY(
    in gl_PerVertex
    {
        vec4 gl_Position;
    } gl_in[];

    out gl_PerVertex
    {
        vec4 gl_Position;
    } gl_out[];

    // 12 13 14 15
    //  8  9 10 11
    //  4  5  6  7
    //  0  1  2  3
    // =>
    // 0  1  2  3, 12 13 14 15,  4  8,  7 11
    layout(vertices = 12) out;

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

        vec2 factor = 0.1 * vec2(viewportMatrix[0][0], viewportMatrix[1][1]);
        vec2 rect = (bounds1.xy - bounds0.xy) * factor;
        return clamp(max(rect.x, rect.y), 4.0, 64.0);
    }

    const int map[12] = int[](0, 1, 2, 3, 12, 13, 14, 15, 4, 8, 7, 11);
    void main()
    {
        if (gl_InvocationID < 12)
            gl_out[gl_InvocationID].gl_Position = gl_in[map[gl_InvocationID]].gl_Position;

        if (gl_InvocationID == 0)
        {
            float value = level();

            gl_TessLevelOuter[0] = 4; // Edges
            gl_TessLevelOuter[1] = level();
        }
    }
);

// Depends on MatrixBlock:
// - mat4 mvpMatrix
static const char* _tesContour = STRINGIFY(
    in gl_PerVertex
    {
        vec4 gl_Position;
    } gl_in[];

    out gl_PerVertex
    {
        vec4 gl_Position;
    };

    layout(isolines) in;

    layout(location = 0) out vec3 v_position;
    layout(location = 1) out vec3 v_normal;
    layout(location = 2) out vec4 v_uv;

    vec4 deCasteljau(float u, inout vec4 p[4])
    {
        p[0] += u * (p[1] - p[0]);
        p[1] += u * (p[2] - p[1]);
        p[2] += u * (p[3] - p[2]);

        p[0] += u * (p[1] - p[0]);
        p[1] += u * (p[2] - p[1]);

        return p[0] + u * (p[1] - p[0]);
    }

    const ivec4 map[4] = ivec4[](
        ivec4( 0, 1, 2, 3),
        ivec4( 4, 5, 6, 7),
        ivec4( 0, 8, 9, 4),
        ivec4( 3,10,11, 7)
    );
    void main()
    {
        int curveId = int(gl_TessCoord.y * 4);

        vec4 C[4] = vec4[](
            gl_in[map[curveId][0]].gl_Position,
            gl_in[map[curveId][1]].gl_Position,
            gl_in[map[curveId][2]].gl_Position,
            gl_in[map[curveId][3]].gl_Position
        );

        vec4 P = deCasteljau(gl_TessCoord.x, C);
        gl_Position = mvpMatrix * P;
        vec4 Q = mvMatrix * P;
        v_position = Q.xyz / Q.w;
        v_uv = mix(vec4(1,0,0,1), vec4(0,1,0,1), gl_TessCoord.x);
    }
);

SurfacePipeline::SurfacePipeline(GLuint vertex, GLuint fragment, Mode mode)
    : _program{mode == Standard3D
        ? "Bézier Tessellation"
        : "Bézier Contour Curves Tessellation"}
{
    _program.setShader(GL_VERTEX_SHADER, {_glslVersion, _vs});

    switch (mode)
    {
    case ContourCurves:
        _program.setShader(GL_TESS_CONTROL_SHADER, {
            _glslVersion,
            GLSL::MATRIX_BLOCK_DECLARATION,
            _tcsContour
        });
        _program.setShader(GL_TESS_EVALUATION_SHADER, {
            _glslVersion,
            GLSL::MATRIX_BLOCK_DECLARATION,
            // GLSL::BICUBIC_DECASTELJAU_FUNCTIONS,
            _tesContour
        });
        break;
    default:
    case Standard3D:
        _program.setShader(GL_TESS_CONTROL_SHADER, {
            _glslVersion,
            GLSL::MATRIX_BLOCK_DECLARATION,
            _tcs
        });
        _program.setShader(GL_TESS_EVALUATION_SHADER, {
            _glslVersion,
            GLSL::MATRIX_BLOCK_DECLARATION,
            // GLSL::BICUBIC_DECASTELJAU_FUNCTIONS,
            GLSL::PATCH_EVAL_FUNCTIONS,
            _tes
        });
        break;
    }
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

    glActiveShaderProgram(_pipeline, fragment);
}

SurfacePipeline::~SurfacePipeline() {}

bool SurfacePrimitive::localIntersect(const Ray3f& ray, Intersection& hit) const
{
    bool b = _bvh->intersect(ray, hit);
    if (b) hit.object = this;
    return b;
}

bool SurfacePrimitive::localIntersect(const Ray3f& ray) const
{
    bool b = _bvh->intersect(ray);
    return b;
}

vec4f SurfacePrimitive::point(const Intersection &hit) const
{
    auto idx = hit.triangleIndex;
    float u = hit.p.x;
    float v = hit.p.y;

    auto p = _bvh->surface();
    auto points = p->points();
    auto patchIdx = p->indices();

    spline::PatchRef patch (points.data(), patchIdx.data(), uint32_t(idx));
    vec4f P = spline::interpolate(patch, u, v);
    return localToWorldMatrix().transform(P);
}

vec3f SurfacePrimitive::normal(const Intersection &hit) const
{
    auto idx = hit.triangleIndex;
    float u = hit.p.x;
    float v = hit.p.y;

    auto p = _bvh->surface();
    auto points = p->points();
    auto patchIdx = p->indices();

    spline::PatchRef patch (points.data(), patchIdx.data(), uint32_t(idx));
    vec3f N = spline::normal(patch, u, v);
    return _normalMatrix.transform(N.versor()).versor();
}

Bounds3f SurfacePrimitive::bounds() const
{
    return {_bvh->bounds(), _localToWorld};
}

void SurfaceMapper::update()
{
    // ???
}

void SurfaceMapper::updateMatrixBlock(GLRenderer& renderer) const
{
    auto c = renderer.camera();
    auto b = static_cast<GLSL::MatrixBlock*>(
        glMapNamedBuffer(renderer.matrixBlock(), GL_WRITE_ONLY)
    );

    mat4f mv = c->worldToCameraMatrix() * _surface->localToWorldMatrix();
    b->mvMatrix = mv;
    b->mvpMatrix = c->projectionMatrix() * mv;
    b->normalMatrix = mat3f{c->worldToCameraMatrix()} * _surface->normalMatrix();
    b->cameraToWorldMatrix = mat3f{c->cameraToWorldMatrix()};

    glUnmapNamedBuffer(renderer.matrixBlock());
}

bool SurfaceMapper::render(GLRenderer& renderer) const
{
    auto pipeline = renderer.pipeline(GLRenderer::Surface);
    if (!pipeline) return false;

    updateMatrixBlock(renderer);

    // Update texture info to ConfigBlock
    int v[3] { 0, 0, 0 };
    if (auto pbr = _surface->pbrMaterial.get())
    {
        auto& s = renderer.fragmentShader()->samplers();
        if (pbr->texBaseColor != nullptr)
        {
            v[0] = 1;
            v[1] = 1;
            glActiveTexture(GL_TEXTURE0 + s.sDiffuse.textureUnit);
            glBindTexture(GL_TEXTURE_2D, *pbr->texBaseColor);
            // specular
            glActiveTexture(GL_TEXTURE0 + s.sSpecular.textureUnit);
            glBindTexture(GL_TEXTURE_2D, *pbr->texBaseColor);
        }
        if (pbr->texMetalRough != nullptr)
        {
            v[2] = 1;
            glActiveTexture(GL_TEXTURE0 + s.sMetalRough.textureUnit);
            glBindTexture(GL_TEXTURE_2D, *pbr->texMetalRough);
        }
    }
    renderer.renderMaterial(*_surface->material());
    glNamedBufferSubData(
        renderer.configBlock(),
        offsetof (GLSL::ConfigBlock, hasDiffuseTexture),
        sizeof (v),
        v
    );

    auto s = _surface->surface();

    pipeline->use();
    // TODO Review if this function call is `SurfaceMapper`'s responsability.
    pipeline->beforeDrawing(renderer);

    s->bind();

    for (auto& g : s->groups())
    {
        if (g.type != PatchType_Bezier)
            continue;
        glPatchParameteri(GL_PATCH_VERTICES, 16);
        glDrawElements(GL_PATCHES, g.size * g.count, GL_UNSIGNED_INT,
            reinterpret_cast<const void*>(g.offset));
    }

    return true;
}

bool SurfaceMapper::renderContour(GLRenderer& renderer) const
{
    auto pipeline = renderer.pipeline("SurfCont"_ID);
    if (!pipeline) return false;

    updateMatrixBlock(renderer);

    auto s = _surface->surface();    

    pipeline->use();
    pipeline->beforeDrawing(renderer);

    s->bind();

    for (auto& g : s->groups())
    {
        if (g.type != PatchType_Bezier)
            continue;
        glPatchParameteri(GL_PATCH_VERTICES, 16);
        glDrawElements(GL_PATCHES, g.size * g.count, GL_UNSIGNED_INT,
            reinterpret_cast<const void*>(g.offset));
    }

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

} // namespace cg
