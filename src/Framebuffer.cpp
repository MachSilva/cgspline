#include "Framebuffer.h"

namespace cg::gl
{

struct GLStateGuard
{
    GLuint drawFramebuffer, texture, textureMS, activeTexture;

    GLStateGuard()
    {
        glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, (GLint*) &drawFramebuffer);
        glGetIntegerv(GL_TEXTURE_BINDING_2D, (GLint*) &texture);
        glGetIntegerv(GL_TEXTURE_BINDING_2D_MULTISAMPLE, (GLint*) &textureMS);
        glGetIntegerv(GL_ACTIVE_TEXTURE, (GLint*) &activeTexture);
    }

    ~GLStateGuard()
    {
        glActiveTexture(activeTexture);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, textureMS);
        glBindTexture(GL_TEXTURE_2D, texture);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, drawFramebuffer);
    }
};

Framebuffer::Framebuffer(const FramebufferDescription* d)
{
    if (d->samples > 32U)
        throw std::runtime_error("Unexpected high number of samples requested");
    
    GLStateGuard _g; // Save state util the end of scope

    _description = *d;

    auto len0 = d->renderbufferCount;
    auto len1 = len0 + d->textureCount;

    _attachments = std::make_unique<AttachmentDescription[]>(len1);
    _description.pRenderbuffers = &_attachments[0];
    _description.pTextures = &_attachments[len0];

    std::copy(d->pRenderbuffers, d->pRenderbuffers + d->renderbufferCount,
        _description.pRenderbuffers);
    std::copy(d->pTextures, d->pTextures + d->textureCount,
        _description.pTextures);

    _renderbuffers = std::make_unique<GLuint[]>(d->renderbufferCount);
    _textures = std::make_unique<GLuint[]>(d->textureCount);

    glGenFramebuffers(1, &_handle);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _handle);

    glGenRenderbuffers(d->renderbufferCount, _renderbuffers.get());
    glGenTextures(d->textureCount, _textures.get());

    GLenum drawBuffers[32U];
    auto drawBuffersLen = 0U;

    for (auto i = 0U; i < d->renderbufferCount; i++)
    {
        const auto& a = d->pRenderbuffers[i];
        glBindRenderbuffer(GL_RENDERBUFFER, _renderbuffers[i]);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER,
            d->samples,
            a.format,
            d->width,
            d->height);
        glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER,
            a.attachment,
            GL_RENDERBUFFER,
            _renderbuffers[i]);
        
        if (a.attachment >= GL_COLOR_ATTACHMENT0
            && a.attachment <= GL_COLOR_ATTACHMENT31)
        {
            drawBuffers[drawBuffersLen] = a.attachment;
            ++drawBuffersLen;
        }
    }

    _textureTarget = _description.samples > 1
        ? GL_TEXTURE_2D_MULTISAMPLE : GL_TEXTURE_2D;

    for (auto i = 0U; i < d->textureCount; i++)
    {
        const auto& a = d->pTextures[i];
        // glActiveTexture(GL_TEXTURE0);
        glBindTexture(_textureTarget, _textures[i]);
        if (_description.samples > 1)
        {
            glTexStorage2DMultisample(_textureTarget,
                d->samples,
                a.format,
                d->width,
                d->height,
                GL_TRUE);
        }
        else
        {
            glTexStorage2D(_textureTarget, 1, a.format, d->width, d->height);
            glTexParameteri(_textureTarget, GL_TEXTURE_MIN_FILTER,
                d->textureMinFilter);
            glTexParameteri(_textureTarget, GL_TEXTURE_MAG_FILTER,
                d->textureMagFilter);
        }
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER,
            a.attachment,
            _textureTarget,
            _textures[i],
            0);

        if (a.attachment >= GL_COLOR_ATTACHMENT0
            && a.attachment <= GL_COLOR_ATTACHMENT31)
        {
            drawBuffers[drawBuffersLen] = a.attachment;
            ++drawBuffersLen;
        }
    }

    glDrawBuffers(drawBuffersLen, drawBuffers);

    _status = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
}

Framebuffer::~Framebuffer()
{
    glDeleteTextures(_description.textureCount, _textures.get());
    glDeleteRenderbuffers(_description.renderbufferCount, _renderbuffers.get());
    glDeleteFramebuffers(1, &_handle);
}

void Framebuffer::resize(uint32_t width, uint32_t height)
{ // FIXME storage is immutable
    GLStateGuard _g; // Save state util the end of scope

    _description.width = width;
    _description.height = height;

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _handle);

    for (auto i = 0U; i < _description.renderbufferCount; i++)
    {
        const auto& a = _description.pRenderbuffers[i];
        glBindRenderbuffer(GL_RENDERBUFFER, _renderbuffers[i]);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER,
            _description.samples,
            a.format,
            width,
            height);
    }

    for (auto i = 0U; i < _description.textureCount; i++)
    {
        const auto& a = _description.pTextures[i];
        glBindTexture(_textureTarget, _textures[i]);
        if (_description.samples > 1)
        {
            glTexStorage2DMultisample(_textureTarget,
                _description.samples,
                a.format,
                width,
                height,
                GL_TRUE);
        }
        else
        {
            glTexStorage2D(_textureTarget, 1, a.format, width, height);
        }
    }

    _status = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
}

} // namespace cg::gl
