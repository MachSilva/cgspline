#include "Texture.h"

#include <cassert>
#include "stb_image.h"

namespace cg::gl
{

// static GLuint g_Texture2D = 0;

// void pushTexture2D()
// {
//     assert(g_Texture2D == 0);
//     glGetIntegerv(GL_TEXTURE_BINDING_2D, (GLuint*) &g_Texture2D);
// }

// GLuint popTexture2D()
// {
//     assert(g_Texture2D);
//     glBindTexture(GL_TEXTURE_2D, g_Texture2D);
//     auto ret = g_Texture2D;
//     g_Texture2D = ret;
//     return ret;
// }

Ref<Texture> Texture::from(const char* filename)
{
    int x, y, components = 0;
    // XXX Open file just once
    if (1 != stbi_info(filename, &x, &y, &components))
        throw std::runtime_error("Could not open/read image");

    // bool is16bit = stbi_is_16_bit(filename);
    bool isHDR = stbi_is_hdr(filename);

    Format internalformat;
    GLenum format, type;
    switch (components)
    {
    case 1:
        if (isHDR)
        {
            format = GL_RED, type = GL_FLOAT,
            internalformat = Format::Grey32F;
        }
        else
        {
            format = GL_RED, type = GL_UNSIGNED_BYTE,
            internalformat = Format::Grey;
        } break;
    case 2:
        if (isHDR)
        {
            format = GL_RG, type = GL_FLOAT,
            internalformat = Format::RGB32F;
        }
        else
        {
            format = GL_RG, type = GL_UNSIGNED_BYTE,
            internalformat = Format::GreyAlpha;
        } break;
    case 3:
        if (isHDR)
        {
            format = GL_RGB, type = GL_FLOAT,
            internalformat = Format::RGB32F;
        }
        else
        {
            format = GL_RGB, type = GL_UNSIGNED_BYTE,
            internalformat = Format::sRGB;
        } break;
    case 4:
        if (isHDR)
        {
            format = GL_RGBA, type = GL_FLOAT,
            internalformat = Format::RGBA32F;
        }
        else
        {
            format = GL_RGBA, type = GL_UNSIGNED_BYTE,
            internalformat = Format::sRGBA;
        } break;
    default:
        throw std::runtime_error("Unexpected read failure");
    }

    void* data = 0;

    if (isHDR)
        data = stbi_loadf(filename, &x, &y, &components, components);
    else
        data = stbi_load(filename, &x, &y, &components, components);

    auto tex = Ref(new Texture(GL_TEXTURE_2D));
    glBindTexture(GL_TEXTURE_2D, *tex);
    glTexStorage2D(GL_TEXTURE_2D, 3, (GLenum) internalformat, x, y);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, x, y, format, type, data);
    glGenerateMipmap(GL_TEXTURE_2D);

    tex->_format = internalformat;
    tex->_width = x;
    tex->_height = y;

    stbi_image_free(data);
    return tex;
}

// Ref<Texture> Texture::from(const uint8_t* buffer, uint32_t length)
// {

// }

Texture::~Texture()
{
    glDeleteTextures(1, &_texture);
}

} // namespace cg::gl
