#pragma once

#include <cassert>
#include <graphics/GLBuffer.h>
#include <memory>
#include "Ref.h"

namespace cg
{

namespace detail
{

struct BufferUnmap
{
    GLuint buffer;
    void operator () (const void*) const
    {
        glUnmapNamedBuffer(buffer);
    }
};

};

template<typename T>
auto mapBufferToUniquePtr(GLuint buffer, GLenum access)
{
    assert(access == GL_READ_ONLY || access == GL_WRITE_ONLY || access == GL_READ_WRITE);
    auto ptr = (T*) glMapNamedBuffer(buffer, access);
    detail::BufferUnmap del (buffer);
    return std::unique_ptr<T[]>(ptr, del);
}

template<typename T>
class GLStorage : public SharedObject
{
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

    auto scopedMap(uint32_t start, uint32_t count, GLenum access = GL_MAP_READ_BIT | GL_MAP_WRITE_BIT)
    {
        detail::BufferUnmap del (_buffer);
        return std::unique_ptr<T[], detail::BufferUnmap>
            (map(start, count, access), del);
    }

    auto scopedMap(GLenum access = GL_READ_ONLY)
    {
        detail::BufferUnmap del (_buffer);
        return std::unique_ptr<T[], detail::BufferUnmap>(map(access), del);
    }

    auto scopedMap() const
    {
        detail::BufferUnmap del (_buffer);
        return std::unique_ptr<const T[], detail::BufferUnmap>(map(), del);
    }

    T* map(uint32_t start, uint32_t count, GLenum access = GL_MAP_READ_BIT | GL_MAP_WRITE_BIT)
    {
        return static_cast<T*>(glMapNamedBufferRange(_buffer, start * sizeof (T), count * sizeof (T), access));
    }

    T* map(GLenum access = GL_READ_ONLY)
    {
        assert(access == GL_READ_ONLY || access == GL_WRITE_ONLY || access == GL_READ_WRITE);
        return static_cast<T*>(glMapNamedBuffer(_buffer, access));
    }

    const T* map() const
    {
        return static_cast<const T*>(glMapNamedBuffer(_buffer, GL_READ_ONLY));
    }

    void unmap() const { glUnmapNamedBuffer(_buffer); }

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
