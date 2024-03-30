#pragma once

#include <graphics/GLProgram.h>
#include <graphics/GLRenderer.h>

namespace cg
{

class SplRenderer : public SharedObject, public GLSL::Program
{
public:
    SplRenderer();
    ~SplRenderer();

    void drawTexture2D(int textureUnit,
        const vec4f quad[4] = nullptr,
        const vec2f uv[4] = nullptr);

    void set_sImage(int textureUnit)
    {
        _sImage.set(textureUnit);
    }

private:
    SamplerData _sImage;
    GLuint _vao;
    GLuint _buffer;
};

} // namespace cg
