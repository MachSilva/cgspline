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
    auto guard(T* ptr) //-> std::unique_ptr<T[],void(T*)>
    {
        assert(ptr);
        auto del = [this](T*) { this->unmap(); };
        return std::unique_ptr<T[],decltype(del)>(ptr, del);
    }

    auto guard(const T* ptr) const //-> std::unique_ptr<const T[],void(const T*)>
    {
        assert(ptr);
        auto del = [this](const T*) { this->unmap(); };
        return std::unique_ptr<const T[],decltype(del)>(ptr, del);
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

    auto map(uint32_t start, uint32_t count, GLenum access = GL_MAP_READ_BIT | GL_MAP_WRITE_BIT)
    {
        return guard(static_cast<T*>(
            glMapNamedBufferRange(_buffer, start*sizeof (T), count*sizeof (T), access)
        ));
    }

    auto map(GLenum access = GL_READ_WRITE)
    {
        assert(access == GL_READ_ONLY || access == GL_WRITE_ONLY || access == GL_READ_WRITE);
        return guard(static_cast<T*>(glMapNamedBuffer(_buffer, access)));
    }

    auto map() const
    {
        return guard(static_cast<const T*>(glMapNamedBuffer(_buffer, GL_READ_ONLY)));
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

    // operator GLStorage<const >

private:
    GLuint _buffer;
    uint32_t _size {0};
    // uint32_t _capacity {0};
};

} // namespace cg
