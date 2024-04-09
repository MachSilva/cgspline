#pragma once

#include <cinttypes>
#include <core/SharedObject.h>
#include "CUDAUtility.h"
#include "RTNamespace.h"
#include "../Ref.h"

namespace cg::rt
{

struct Frame : SharedObject
{
    using Pixel = uint32_t;

    Frame() = default;
    Frame(uint32_t width, uint32_t height,
        memory_resource* mr = std::pmr::get_default_resource())
        : _width{width}, _height{height}
        , _data((size_t) width * height, mr)
    {}

    __host__ __device__
    const Pixel& at(uint32_t x, uint32_t y) const
    {
        return _data[_width * y + x];
    }

    __host__ __device__
    Pixel& at(uint32_t x, uint32_t y)
    {
        return _data[_width * y + x];
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
    auto size_bytes() const { return _data.size_bytes(); }

protected:
    Buffer<Pixel> _data;
    uint32_t _height {};
    uint32_t _width {};
};

} // namespace cg::rt
