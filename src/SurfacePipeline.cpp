#include "SurfacePipeline.h"

#include <graphics/GLRenderer.h>
#include <graph/Scene.h>
#include "Log.h"
#include "Spline.h"
#include "SplineMat.h"

#define TOSTR(s) #s

namespace cg
{

namespace GLSL
{

const char * const BICUBIC_DECASTELJAU_FUNCTIONS = TOSTR(
    vec4 _p[4];
    vec4 _q[4];
    const int _degU = 3;
    const int _degV = 3;

    vec4 point(int i);
    vec4 pointAt(int i, int j)
    {
        return point(4*j + i);
    }

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

const char * const BEZIER_PATCH_EVAL_FUNCTIONS = TOSTR(
    vec4 point(int i);

    // Evaluate bezier basis function B(u) only
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

    // Evaluate bezier basis function: B(u)
    // in  u: coordinate
    // out b: basis function B(u) values per control point
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
    void evalBezierSurface(float u, float v, out vec3 s[3])
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
        for (int j = 0, k = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                const vec3 P = point(k++).xyz;
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
            for (int j = 0, k = 0; j < 4; j++)
                for (int i = 0; i < 4; i++)
                    s[1] += (bdu[i] * bv[j]) * point(k++).xyz;
        }
        if (length(s[2]) <= 1e-6)
        {
            s[2] = vec3(0);
            bu = evalBezierBasis(abs(u - fixValue));
            for (int j = 0, k = 0; j < 4; j++)
                for (int i = 0; i < 4; i++)
                    s[2] += (bu[i] * bdv[j]) * point(k++).xyz;
        }
    }
);

const char * const BSPLINE_PATCH_EVAL_FUNCTIONS = TOSTR(
    vec4 point(int i);

    // Evaluate b-spline basis function: B(u)
    // in  u: coordinate
    // out b: basis function B(u) values per control point
    //     d: derivative function B'(u) values
    void evalBSpline(float u, out vec4 b, out vec4 d)
    {
        const float u2 = u*u;
        const float u3 = u2*u;
        const float _3u2 = 3*u2;
        b = (1.0/6.0) * vec4(
            1 - 3*u + _3u2 - u3,
            4       - 6*u2 + 3*u3,
            1 + 3*u + _3u2 - 3*u3,
            u3
        );
        d = vec4(
            -0.5 + u - 0.5*u2,
            -2*u + 1.5*u2,
            0.5 + u - 1.5*u2,
            0.5*u2
        );
    }

    // Evaluate surface at point (u,v)
    // out s:
    //     s[0]: S(u,v)
    //     s[1]: S'u(u,v)
    //     s[2]: S'v(u,v)
    void evalBSplineSurface(float u, float v, out vec3 s[3])
    {
        vec4 bu;
        vec4 bdu;
        vec4 bv;
        vec4 bdv;
        evalBSpline(u, bu, bdu);
        evalBSpline(v, bv, bdv);
        s[0] = vec3(0);
        s[1] = vec3(0);
        s[2] = vec3(0);
        for (int j = 0, k = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                const vec3 P = point(k++).xyz;
                s[0] += (bu[i] * bv[j]) * P;
                s[1] += (bdu[i] * bv[j]) * P;
                s[2] += (bu[i] * bdv[j]) * P;
            }
        }
    }
);

/**
 * Depends on:
 * - evalBezier() (available in BEZIER_PATCH_EVAL_FUNCTIONS)
 * - point()
 * 
 * Provides:
 * - void evalGregory(float u, float v, out vec4 g, out vec4 gdu, out vec4 gdv);
 * - void evalGregorySurface(float u, float v, out vec3 s[3]);
 * - const int U_IDX[20], V_IDX[20], OUTER[12], INNER[8], G_IDX[8], G_SIGN[8];
 *
 * Gregory patch control point disposition:
 * 
 * 15 17      11 10
 * 16   18  14   12
 *    19      13
 *     3       9
 *  2    4   8    6
 *  0  1       7  5
 * 
 */
const char * const GREGORY_PATCH_EVAL_FUNCTIONS = TOSTR(
    vec4 point(int i);
    void evalBezier(float u, out vec4 b, out vec4 d);

    void evalGregory(float u, float v, out vec4 g, out vec4 gdu, out vec4 gdv)
    {
        const float ui = 1 - u;
        const float vi = 1 - v;
        vec4 q = 1.0 / vec4(u+v, ui+v, u+vi, ui+vi);
        g = vec4(v, v, vi, vi) * q;
        q = q * q;
        gdu = vec4(-v, v, -vi, vi) * q;
        gdv = vec4(u, ui, -u, -ui) * q;
    }

    // Gregory patch control point disposition:
    // 15 17      11 10
    // 16   18  14   12
    //    19      13
    //     3       9
    //  2    4   8    6
    //  0  1       7  5
    const int U_IDX[20] = int[](0,1,0,1,1,3,3,2,2,2,3,2,3,2,2,0,0,1,1,1);
    const int V_IDX[20] = int[](0,0,1,1,1,0,1,0,1,1,3,3,2,2,2,3,2,3,2,2);
    const int OUTER[12] = int[](0,1,2,5,6,7,10,11,12,15,16,17);
    const int INNER[8] = int[](3,4,8,9,13,14,18,19);
    const int G_IDX[8] = int[](0,0,1,1,3,3,2,2);
    const int G_SIGN[8] = int[](+1,-1,-1,+1,+1,-1,-1,+1);

    // Evaluate gregory surface point: S(u,v)
    // out s:
    //     s[0]: S(u,v)
    //     s[1]: S'u(u,v)
    //     s[2]: S'v(u,v)
    void evalGregorySurface(float u, float v, out vec3 s[3])
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
        for (int k = 0; k < 12; k++)
        {
            const int i = U_IDX[OUTER[k]];
            const int j = V_IDX[OUTER[k]];
            const vec3 P = point(k).xyz;
            s[0] += (bu[i] * bv[j]) * P;
            s[1] += (bdu[i] * bv[j]) * P;
            s[2] += (bu[i] * bdv[j]) * P;
        }
        vec4 g;
        vec4 gdu;
        vec4 gdv;
        evalGregory(u, v, g, gdu, gdv);
        for (int k = 0; k < 8; k++)
        {
            const int i = U_IDX[INNER[k]];
            const int j = V_IDX[INNER[k]];
            const int h = G_IDX[k];
            const int sign = G_SIGN[k];
            const float suv = bu[i] * bv[j];
            const vec3 P = point(k).xyz;
            s[0] += (sign * suv * g[h]) * P;
            s[1] += ((bdu[i] * bv[j] * g[h] + suv * gdu[h]) * sign) * P;
            s[2] += ((bu[i] * bdv[j] * g[h] + suv * gdv[h]) * sign) * P;
        }
    }
);

/**
 *
 * Provides:
 * - float level();
 *
 * It depends on MatrixBlock:
 * - mvpMatrix
 * - viewportMatrix
 * It reads from gl_out instead of gl_in, so you MUST write to gl_out and then
 * call barrier() before calling level().
 *
 */
const char* const TCS_GL_OUT_LEVEL_FUNCTION = TOSTR(
    float level()
    {
        const vec3 limit = vec3(1.0);
        vec3 bounds0 = vec3(2.0);
        vec3 bounds1 = vec3(-2.0);

        for (int i = 0; i < gl_out.length(); i++)
        {
            vec4 p = mvpMatrix * gl_out[i].gl_Position;
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
);

const char* const ELEMENT_MATRIX_BUFFER_DECL = TOSTR(
    layout(std430) readonly restrict buffer ElementMatrixBuffer
    {
        float elementMatrixBuffer[];
    };
    // uniform bool useElementMatrix = false;
    uniform int elementMatrixOffset = -1;
);

} // namespace GLSL

static const char* _glslVersion = "#version 430 core\n";

static const char* _vs = TOSTR(
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

// Depends on:
// - MatrixBlock: mvpMatrix
// - float level()
// Requires layout qualifier `vertices` to be set
static const char* _tcs = TOSTR(
    in gl_PerVertex
    {
        vec4 gl_Position;
    } gl_in[];

    out gl_PerVertex
    {
        vec4 gl_Position;
    } gl_out[];

    float level();

    void main()
    {
        if (elementMatrixOffset < 0)
        {
            gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
        }
        else
        {
            uint offset = elementMatrixOffset
                + gl_PrimitiveID * gl_PatchVerticesIn * gl_out.length() // matrix start
                + gl_InvocationID; // line start
            vec4 P = vec4(0);
            for (uint i = 0; i < gl_PatchVerticesIn; i++)
            {
                P += elementMatrixBuffer[offset] * gl_in[i].gl_Position;
                offset += gl_out.length();
            }
            gl_out[gl_InvocationID].gl_Position = P;
        }
        barrier();
        if (gl_InvocationID == 0)
        {
            float value = level();

            gl_TessLevelOuter[0] = gl_TessLevelOuter[1] =
            gl_TessLevelOuter[2] = gl_TessLevelOuter[3] = 
            gl_TessLevelInner[0] = gl_TessLevelInner[1] = value;
        }
    }
);

static const char* _contourTCS = TOSTR(
    in gl_PerVertex
    {
        vec4 gl_Position;
    } gl_in[];

    out gl_PerVertex
    {
        vec4 gl_Position;
    } gl_out[];

    float level();

    void main()
    {
        if (elementMatrixOffset < 0)
        {
            gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
        }
        else
        {
            uint offset = elementMatrixOffset
                + gl_PrimitiveID * gl_PatchVerticesIn * gl_out.length()
                + gl_InvocationID;
            vec4 P = vec4(0);
            for (uint i = 0; i < gl_PatchVerticesIn; i++)
            {
                P += elementMatrixBuffer[offset] * gl_in[i].gl_Position;
                offset += gl_out.length();
            }
            gl_out[gl_InvocationID].gl_Position = P;
        }
        barrier();
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
// - mat4 mvMatrix
static const char* _tes = TOSTR(
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

    vec4 point(int i)
    {
        return gl_in[i].gl_Position;
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

// Depends on MatrixBlock:
// - mat4 mvpMatrix
// - mat4 mvMatrix
static const char* _contourTES = TOSTR(
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

    vec4 point(int i)
    {
        return gl_in[i].gl_Position;
    }

    void eval(float u, float v, out vec3 s[3]);

    vec2 mapCoordinates(int id, float u)
    {
        switch (id)
        {
        default:
        case 0:
            return vec2(u, 0);
        case 1:
            return vec2(u, 1);
        case 2:
            return vec2(0, u);
        case 3:
            return vec2(1, u);
        }
    }
    void main()
    {
        const int curveId = int(gl_TessCoord.y * 4);
        const vec2 coord = mapCoordinates(curveId, gl_TessCoord.x);
        vec4 P;
        {
            vec3 S[3];
            eval(coord.x, coord.y, S);
            P = vec4(S[0], 1.0);
        }
        gl_Position = mvpMatrix * P;
        vec4 Q = mvMatrix * P;
        v_position = Q.xyz / Q.w;
        v_uv = mix(vec4(1,0,0,1), vec4(0,1,0,1), gl_TessCoord.x);
    }
);

static constexpr const char* const k_ProgramNames[]
{
    "Bézier Tessellation",
    "B-Spline Tessellation",
    "Gregory Tessellation",
    "Bézier Contour Curves Tessellation",
    "B-Spline Contour Curves Tessellation",
    "Gregory Contour Curves Tessellation",
};

static constexpr const char* const _eval_BezierSurface =
"void eval(float u, float v, out vec3 s[3]) { evalBezierSurface(u, v, s); }";
static constexpr const char* const _eval_BSplineSurface =
"void eval(float u, float v, out vec3 s[3]) { evalBSplineSurface(u, v, s); }";
static constexpr const char* const _eval_GregorySurface =
"void eval(float u, float v, out vec3 s[3]) { evalGregorySurface(u, v, s); }";

static const char* const _layout16 = "layout(vertices = 16) out;";
static const char* const _layout20 = "layout(vertices = 20) out;";

SurfacePipeline::SurfacePipeline(GLuint vertex, GLuint fragment)
{
    _passthroughVS = new GLSL::ShaderProgram
        (GL_VERTEX_SHADER, {_glslVersion, _vs});

    auto check = [](GLSL::ShaderProgram* p)
    {
        if (!p->ok())
            throw std::runtime_error
                ("failed to create shader program\n" + p->infoLog());
    };

    _Patch16TCS.program = new GLSL::ShaderProgram(GL_TESS_CONTROL_SHADER, {
        _glslVersion,
        _layout16,
        GLSL::MATRIX_BLOCK_DECLARATION,
        GLSL::ELEMENT_MATRIX_BUFFER_DECL,
        _tcs,
        GLSL::TCS_GL_OUT_LEVEL_FUNCTION,
    });
    check(_Patch16TCS.program);
    _Patch20TCS.program = new GLSL::ShaderProgram(GL_TESS_CONTROL_SHADER, {
        _glslVersion,
        _layout20,
        GLSL::MATRIX_BLOCK_DECLARATION,
        GLSL::ELEMENT_MATRIX_BUFFER_DECL,
        _tcs,
        GLSL::TCS_GL_OUT_LEVEL_FUNCTION,
    });
    check(_Patch20TCS.program);

    _BezierTES = new GLSL::ShaderProgram(GL_TESS_EVALUATION_SHADER, {
        _glslVersion,
        GLSL::MATRIX_BLOCK_DECLARATION,
        GLSL::BEZIER_PATCH_EVAL_FUNCTIONS,
        _eval_BezierSurface,
        _tes
    });
    check(_BezierTES);
    _BSplineTES = new GLSL::ShaderProgram(GL_TESS_EVALUATION_SHADER, {
        _glslVersion,
        GLSL::MATRIX_BLOCK_DECLARATION,
        GLSL::BSPLINE_PATCH_EVAL_FUNCTIONS,
        _eval_BSplineSurface,
        _tes
    });
    check(_BSplineTES);
    _GregoryTES = new GLSL::ShaderProgram(GL_TESS_EVALUATION_SHADER, {
        _glslVersion,
        GLSL::MATRIX_BLOCK_DECLARATION,
        GLSL::BEZIER_PATCH_EVAL_FUNCTIONS,
        GLSL::GREGORY_PATCH_EVAL_FUNCTIONS,
        _eval_GregorySurface,
        _tes
    });
    check(_GregoryTES);

    _Patch16ContourTCS.program = new GLSL::ShaderProgram(GL_TESS_CONTROL_SHADER, {
        _glslVersion,
        _layout16,
        GLSL::MATRIX_BLOCK_DECLARATION,
        GLSL::ELEMENT_MATRIX_BUFFER_DECL,
        _contourTCS,
        GLSL::TCS_GL_OUT_LEVEL_FUNCTION,
    });
    check(_Patch16ContourTCS.program);
    _Patch20ContourTCS.program = new GLSL::ShaderProgram(GL_TESS_CONTROL_SHADER, {
        _glslVersion,
        _layout20,
        GLSL::MATRIX_BLOCK_DECLARATION,
        GLSL::ELEMENT_MATRIX_BUFFER_DECL,
        _contourTCS,
        GLSL::TCS_GL_OUT_LEVEL_FUNCTION,
    });
    check(_Patch20ContourTCS.program);
    _BezierContourTES = new GLSL::ShaderProgram(GL_TESS_EVALUATION_SHADER, {
        _glslVersion,
        GLSL::MATRIX_BLOCK_DECLARATION,
        GLSL::BEZIER_PATCH_EVAL_FUNCTIONS,
        _eval_BezierSurface,
        _contourTES
    });
    check(_BezierContourTES);
    _BSplineContourTES = new GLSL::ShaderProgram(GL_TESS_EVALUATION_SHADER, {
        _glslVersion,
        GLSL::MATRIX_BLOCK_DECLARATION,
        GLSL::BSPLINE_PATCH_EVAL_FUNCTIONS,
        _eval_BSplineSurface,
        _contourTES
    });
    check(_BSplineContourTES);
    _GregoryContourTES = new GLSL::ShaderProgram(GL_TESS_EVALUATION_SHADER, {
        _glslVersion,
        GLSL::MATRIX_BLOCK_DECLARATION,
        GLSL::BEZIER_PATCH_EVAL_FUNCTIONS,
        GLSL::GREGORY_PATCH_EVAL_FUNCTIONS,
        _eval_GregorySurface,
        _contourTES
    });
    check(_GregoryContourTES);

    // constexpr auto name0 = "useElementMatrix";
    // _Patch16TCS.useElementMatrixLoc =
    //     glGetUniformLocation(*_Patch16TCS.program, name0);
    // _Patch20TCS.useElementMatrixLoc =
    //     glGetUniformLocation(*_Patch20TCS.program, name0);

    constexpr auto name1 = "ElementMatrixBuffer";
    _Patch16TCS.elementMatrixBufferIdx = glGetProgramResourceIndex
        (*_Patch16TCS.program, GL_SHADER_STORAGE_BLOCK, name1);
    _Patch20TCS.elementMatrixBufferIdx = glGetProgramResourceIndex
        (*_Patch20TCS.program, GL_SHADER_STORAGE_BLOCK, name1);
    _Patch16ContourTCS.elementMatrixBufferIdx = glGetProgramResourceIndex
        (*_Patch16ContourTCS.program, GL_SHADER_STORAGE_BLOCK, name1);
    _Patch20ContourTCS.elementMatrixBufferIdx = glGetProgramResourceIndex
        (*_Patch20ContourTCS.program, GL_SHADER_STORAGE_BLOCK, name1);

    constexpr auto name3 = "elementMatrixOffset";
    _Patch16TCS.elementMatrixOffsetLoc =
        glGetUniformLocation(*_Patch16TCS.program, name3);
    _Patch20TCS.elementMatrixOffsetLoc =
        glGetUniformLocation(*_Patch20TCS.program, name3);
    _Patch16ContourTCS.elementMatrixOffsetLoc =
        glGetUniformLocation(*_Patch16ContourTCS.program, name3);
    _Patch20ContourTCS.elementMatrixOffsetLoc =
        glGetUniformLocation(*_Patch20ContourTCS.program, name3);

    // create pipelines
    if (!vertex) vertex = (*_passthroughVS);

    auto setStages = [&](GLuint pipeline, GLuint tcs, GLuint tes)
    {
        glUseProgramStages(pipeline, GL_VERTEX_SHADER_BIT, vertex);
        glUseProgramStages(pipeline, GL_FRAGMENT_SHADER_BIT, fragment);
        glUseProgramStages(pipeline, GL_TESS_CONTROL_SHADER_BIT, tcs);
        glUseProgramStages(pipeline, GL_TESS_EVALUATION_SHADER_BIT, tes);
        glActiveShaderProgram(pipeline, fragment);
    };

    setStages(_pipeline, *_Patch16TCS.program, *_BezierTES);

    glCreateProgramPipelines(std::size(_extraPipelines), _extraPipelines);
    setStages(_extraPipelines[0], *_Patch16TCS.program, *_BSplineTES);
    setStages(_extraPipelines[1], *_Patch20TCS.program, *_GregoryTES);
    setStages(_extraPipelines[2], *_Patch16ContourTCS.program, *_BezierContourTES);
    setStages(_extraPipelines[3], *_Patch16ContourTCS.program, *_BSplineContourTES);
    setStages(_extraPipelines[4], *_Patch20ContourTCS.program, *_GregoryContourTES);
}

SurfacePipeline::~SurfacePipeline()
{
    glDeleteProgramPipelines(std::size(_extraPipelines), _extraPipelines);
}

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
    auto p = dynamic_cast<SurfacePipeline*>(renderer.pipeline(GLRenderer::Surface));
    if (!p) return false;

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

    // TODO Review if this function call is `SurfaceMapper`'s responsability.
    p->beforeDrawing(renderer);

    s->bind();
    auto m = s->matrices();

    constexpr auto kBindingPoint = 0;
    auto& p16 = p->getPatch16TCS();
    auto& p20 = p->getPatch20TCS();
    glShaderStorageBlockBinding(*p16.program, p16.elementMatrixBufferIdx, kBindingPoint);
    glShaderStorageBlockBinding(*p20.program, p20.elementMatrixBufferIdx, kBindingPoint);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, kBindingPoint, m ? m->buffer() : 0);

    glUseProgram(0);
    for (const auto& g : s->groups())
    {
        switch (g.type)
        {
        case PatchType_Gregory:
            glBindProgramPipeline(p->get(SurfacePipeline::eGregoryElement));
            glProgramUniform1i(*p20.program, p20.elementMatrixOffsetLoc, g.matrixOffset);
            break;
        case PatchType_BSpline:
            glBindProgramPipeline(p->get(SurfacePipeline::eBSplineElement));
            glProgramUniform1i(*p16.program, p16.elementMatrixOffsetLoc, g.matrixOffset);
            break;
        case PatchType_Bezier:
            glBindProgramPipeline(p->get(SurfacePipeline::eBezierElement));
            glProgramUniform1i(*p16.program, p16.elementMatrixOffsetLoc, g.matrixOffset);
            break;
        default:
            log::warn("unknown patch type {}", int(g.type));
            return false;
        }
        glPatchParameteri(GL_PATCH_VERTICES, g.size);
        glDrawElements(GL_PATCHES, g.size * g.count, GL_UNSIGNED_INT,
            (uint32_t*)0 + g.offset);
    }

    return true;
}

bool SurfaceMapper::renderContour(GLRenderer& renderer) const
{
    auto p = dynamic_cast<SurfacePipeline*>(renderer.pipeline(GLRenderer::Surface));
    if (!p) return false;

    updateMatrixBlock(renderer);

    auto s = _surface->surface();    
    s->bind();

    auto m = s->matrices();
    constexpr auto kBindingPoint = 0;
    auto& p16 = p->getPatch16ContourTCS();
    auto& p20 = p->getPatch20ContourTCS();
    glShaderStorageBlockBinding(*p16.program, p16.elementMatrixBufferIdx, kBindingPoint);
    glShaderStorageBlockBinding(*p20.program, p20.elementMatrixBufferIdx, kBindingPoint);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, kBindingPoint, m ? m->buffer() : 0);

    glUseProgram(0);
    for (auto& g : s->groups())
    {
        switch (g.type)
        {
        case PatchType_Gregory:
            glBindProgramPipeline(p->get(SurfacePipeline::eGregoryContour));
            glProgramUniform1i(*p20.program, p20.elementMatrixOffsetLoc, g.matrixOffset);
            break;
        case PatchType_BSpline:
            glBindProgramPipeline(p->get(SurfacePipeline::eBSplineContour));
            glProgramUniform1i(*p16.program, p16.elementMatrixOffsetLoc, g.matrixOffset);
            break;
        case PatchType_Bezier:
            glBindProgramPipeline(p->get(SurfacePipeline::eBezierContour));
            glProgramUniform1i(*p16.program, p16.elementMatrixOffsetLoc, g.matrixOffset);
            break;
        default:
            log::warn("unknown patch type {}", int(g.type));
            return false;
        }
        glPatchParameteri(GL_PATCH_VERTICES, g.size);
        glDrawElements(GL_PATCHES, g.size * g.count, GL_UNSIGNED_INT, (uint32_t*)0 + g.offset);
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
