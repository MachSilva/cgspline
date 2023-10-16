#pragma once

#include <core/SharedObject.h>
#include <graphics/GLBuffer.h>
#include <memory>

namespace cg::ext
{

struct AttachmentDescription
{
    GLenum attachment;
    GLenum format;
    // GLenum colorEncoding; // GL_LINEAR or GL_SRGB
};

struct FramebufferDescription
{
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t samples = 0;
    AttachmentDescription* pRenderbuffers = nullptr;
    uint16_t renderbufferCount = 0;
    AttachmentDescription* pTextures = nullptr;
    uint16_t textureCount = 0;
    GLenum textureMinFilter = GL_NEAREST;
    GLenum textureMagFilter = GL_NEAREST;
};

class Framebuffer : public SharedObject
{
protected:
    GLenum _status;
    GLuint _handle;
    FramebufferDescription _description;
    std::unique_ptr<GLuint[]> _renderbuffers;
    std::unique_ptr<GLuint[]> _textures;
    std::unique_ptr<AttachmentDescription[]> _attachments;
    GLenum _textureTarget;

public:
    Framebuffer(const FramebufferDescription* description);
    ~Framebuffer() override;

    void resize(uint32_t width, uint32_t height);

    auto width() const { return _description.width; }
    auto height() const { return _description.height; }

    auto status() const { return _status; }

    auto handle() const { return _handle; }
    operator GLuint () const { return _handle; }
};

} // namespace cg::ext
