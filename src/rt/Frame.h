#pragma once

#include <cinttypes>
#include <core/SharedObject.h>
#include "CUDAUtility.h"
#include "RTNamespace.h"
#include "../Ref.h"

namespace cg::rt
{

/**
 * @brief ...
 * 
 * @param n Value to be aligned
 * @param alignment An alignment value that is not a power of two results in
 *        undefined behaviour
 */
constexpr uint32_t alignTo(uint32_t n, uint32_t alignment)
{
    if (!alignment)
        alignment = 1;
    assert(std::popcount(alignment) == 1);
    auto mask = alignment - 1;
    auto rem = n & mask;
    return rem ? (n + (alignment - rem)) : n; 
}

struct Frame : SharedObject
{
    using Pixel = uint32_t;

    Frame() = default;
    Frame(uint32_t width, uint32_t height,
        memory_resource* mr = std::pmr::get_default_resource())
        : _width{width}, _height{height}, _stride{width}
        , _data((size_t) width * height, mr)
    {}

    /**
     * @brief Construct a new Frame object
     * 
     * @param width 
     * @param height 
     * @param alignment An alignment that is not a power of two results in
     *        undefined behaviour
     * @param mr Memory resource
     */
    Frame(uint32_t width, uint32_t height, uint32_t alignment,
        memory_resource* mr)
        : Frame(alignTo(width, alignment), height, mr)
    {
        _width = width;
    }

    __host__ __device__
    const Pixel& at(uint32_t x, uint32_t y) const
    {
        return _data[_stride * y + x];
    }

    __host__ __device__
    Pixel& at(uint32_t x, uint32_t y)
    {
        return _data[_stride * y + x];
    }

    __host__ __device__
    const Pixel* data() const { return _data.data(); }

    __host__ __device__
    Pixel* data() { return _data.data(); }

    __host__ __device__
    auto height() const { return _height; }

    __host__ __device__
    auto width() const { return _width; }

    __host__ __device__
    auto stride() const { return _stride; }

    __host__ __device__
    auto size() const { return _stride * _height; }

    __host__ __device__
    auto size_bytes() const { return _data.size_bytes(); }

protected:
    Buffer<Pixel> _data;
    uint32_t _height {};
    uint32_t _width {};
    uint32_t _stride {};
};

} // namespace cg::rt
