#pragma once

#include <graphics/GLBuffer.h>
#include "stb_image.h"
#include "Ref.h"

namespace cg::gl
{

// void pushTexture2D();
// GLuint popTexture2D();

// Alpha components are always linear.
enum struct Format : GLint
{
    Unknown = 0,
    // 8-bit linear unsigned int component
    Grey = GL_R8,
    // 8-bit linear unsigned int per component
    GreyAlpha = GL_RG8,
    // 16-bit linear float component
    Grey16F = GL_R16F,
    // 16-bit linear float per component
    GreyAlpha16F = GL_RG16F,
    // 32-bit linear float component
    Grey32F = GL_R32F,
    // 32-bit linear float per component
    GreyAlpha32F = GL_RG32F,
    // 8-bit non-linear (sRGB encoded) unsigned int per component
    sRGB = GL_SRGB8,
    // 8-bit non-linear (sRGB encoded) unsigned int per component
    sRGBA = GL_SRGB8_ALPHA8,
    // 16-bit linear float per component
    RGB16F = GL_RGB16F,
    // 16-bit linear float per component
    RGBA16F = GL_RGBA16F,
    // 32-bit linear float per component
    RGB32F = GL_RGB32F,
    // 32-bit linear float per component
    RGBA32F = GL_RGBA32F
};

class Texture : public cg::SharedObject
{
public:
    // static Ref<Texture> from(const uint8_t* buffer, uint32_t length);
    static Ref<Texture> from(const char* filename);

    Texture(GLenum target = GL_TEXTURE_2D) : _target{target}
    {
        glGenTextures(1, &_texture);
    }

    Texture(Format pixelFormat, uint32_t width, uint32_t height)
        : _target{GL_TEXTURE_2D}, _format{pixelFormat}
        , _width{width}, _height{height}
    {
        glGenTextures(1, &_texture);
        glBindTexture(_target, _texture);
        glTexStorage2D(_target, 1, (GLenum) _format, _width, _height);
    }

    // Texture(GLenum target, );

    Texture(Texture&&) = default;
    Texture(const Texture&) = delete;
    ~Texture() override;

    void bind() const { glBindTexture(_target, _texture); }

    auto format() const { return _format; }
    auto target() const { return _target; }
    auto width() const { return _width; }
    auto height() const { return _height; }

    auto handle() const { return _texture; }
    operator GLuint () const { return _texture; }

protected:
    GLuint _texture;
    GLenum _target;
    uint32_t _width = 0;
    uint32_t _height = 0;
    Format _format;
};

} // namespace cg::gl
