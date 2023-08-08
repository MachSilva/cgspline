#pragma once

#include <cassert>
#include <core/SharedObject.h>
#include <graphics/GLBuffer.h>
#include <memory>

namespace cg
{

template<typename T>
class GLStorage : public SharedObject
{
    auto guard(T* ptr)
    {
        assert(ptr);
        auto del = [this](T*) { this->unmap(); };
        return std::unique_ptr<T[],decltype(del)>(ptr, del);
    }

public:
    GLStorage() { glCreateBuffers(1, &_buffer); }

    GLStorage(uint32_t size, const T* data = nullptr, GLenum usage = GL_DYNAMIC_DRAW) : _size{size}
    {
        glCreateBuffers(1, &_buffer);
        glNamedBufferData(_buffer, size * sizeof (T), data, usage);
    }

    GLStorage(GLStorage&&) = default;
    GLStorage(const GLStorage&) = delete; // To be implemented...

    ~GLStorage() override { glDeleteBuffers(1, &_buffer); }

    auto map(uint32_t start, uint32_t count, GLenum access = GL_READ_WRITE)
    {
        return guard(static_cast<T*>(
            glMapNamedBufferRange(_buffer, start*sizeof (T), count*sizeof (T), access)
        ));
    }

    auto map(GLenum access = GL_READ_WRITE)
    {
        return guard(static_cast<T*>(glMapNamedBuffer(_buffer, access)));
    }

    void unmap() { glUnmapNamedBuffer(_buffer); }

    void resize(uint32_t size, GLenum usage = GL_DYNAMIC_DRAW)
    {
        glNamedBufferData(_buffer, size * sizeof (T), nullptr, usage);
        _size = size;
    }

    auto size() const { return _size; }

    void setData(uint32_t start, uint32_t count, const T* data)
    {
        glNamedBufferSubData(_buffer, start * sizeof (T), count * sizeof (T), data);
    }

    void setData(const T* data)
    {
        setData(0, _size, data);
    }

    operator GLuint () const { return _buffer; }
    auto buffer() const { return _buffer; }

private:
    GLuint _buffer;
    uint32_t _size {0};
    // uint32_t _capacity {0};
};

} // namespace cg
