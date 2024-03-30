#include "SplRenderer.h"
#include <graphics/GLBuffer.h>

#define TOSTR(x) #x

namespace cg
{

static const auto gVertex = TOSTR(
    layout(location = 0) in vec4 position;
    layout(location = 1) in vec2 coord;

    out vec2 v_uv;

    void main()
    {
        gl_Position = position;
        v_uv = coord;
    }
);

static const auto gFragment = TOSTR(
    uniform sampler2D sImage;
    in vec2 v_uv;
    out vec4 color;
    void main()
    {
        color = texture(sImage, v_uv);
    }
);

struct Vertex
{
    vec4f position;
    vec2f uv;
};

SplRenderer::SplRenderer()
    : GLSL::Program("CG::SPLINE Renderer")
{
    setShader(GL_VERTEX_SHADER, {
        "#version 430 core\n",
        gVertex
    });
    setShader(GL_FRAGMENT_SHADER, {
        "#version 430 core\n",
        gFragment
    });
    use();

    _sImage.location = glGetUniformLocation(*this, "sImage");
    _sImage.set(0);

    glGenVertexArrays(1, &_vao);
    glGenBuffers(1, &_buffer);

    glBindVertexArray(_vao);
    glBindBuffer(GL_ARRAY_BUFFER, _buffer);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof (Vertex), (void*) 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE,
        sizeof (Vertex), (void*) offsetof (Vertex, uv));
    glEnableVertexAttribArray(1);
}

SplRenderer::~SplRenderer()
{
    glDeleteBuffers(1, &_buffer);
    glDeleteVertexArrays(1, &_vao);
}

void SplRenderer::drawTexture2D(int textureUnit, const vec4f quad[4],
    const vec2f uv[4])
{
    static const vec4f defaultQuad[]
    {
        {-1, -1, 0, 1}, {1, -1, 0, 1}, {-1, 1, 0, 1}, {1, 1, 0, 1}
    };
    static const vec2f defaultUV[]
    {
        {0, 0}, {1, 0}, {0, 1}, {1, 1}
    };
    Vertex data[6];

    if (!quad)
    {
        quad = defaultQuad;
        if (!uv)
            uv = defaultUV;
    }

    data[0] = { quad[0], uv[0] };
    data[1] = { quad[1], uv[1] };
    data[2] = { quad[2], uv[2] };

    data[3] = { quad[1], uv[1] };
    data[4] = { quad[2], uv[2] };
    data[5] = { quad[3], uv[3] };

    glBindBuffer(GL_ARRAY_BUFFER, _buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof (data), data, GL_STREAM_DRAW);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    use(); // program
    set_sImage(textureUnit);

    glBindVertexArray(_vao);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

} // namespace cg
