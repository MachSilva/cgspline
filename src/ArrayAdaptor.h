#pragma once

#include <cuda/std/array>
#include <cuda/std/cassert>
#include <cuda/std/span>

#include "AlignedTypes.h"

namespace cg
{

namespace custd = ::cuda::std;

namespace detail
{

template<typename T, typename... Args>
_SPL_CONSTEXPR T& construct_at(T* p, Args... args)
{
    ::new (p) T(std::forward<Args>(args)...);
}

template<typename T>
_SPL_CONSTEXPR void destroy_at(T* p) noexcept
{
    p->~T();
}

template<typename T>
_SPL_CONSTEXPR void destroy_n(T* p, int n) noexcept
{
    for (int i = 0; i < n; i++)
        destroy_at(p+i);
}

template<typename T>
_SPL_CONSTEXPR void destroy(T* p, T* q) noexcept
{
    for (; p < q; p++)
        destroy_at(p);
}

template<typename T, typename SizeT, typename DataT = T*>
class ArrayBase
{
public:
    using value_type = T;
    using size_type = SizeT;

    _SPL_CONSTEXPR
    void clear()
    {
        detail::destroy_n(_data, _size);
        _size = 0;
    }

    _SPL_CONSTEXPR
    bool empty() const { return _size == 0; }

    _SPL_CONSTEXPR
    const T& front() const { return _data[0]; }

    _SPL_CONSTEXPR
    const T& back() const { return _data[_size - 1]; }

    _SPL_CONSTEXPR
    T& front() { return _data[0]; }

    _SPL_CONSTEXPR
    T& back() { return _data[_size - 1]; }

    _SPL_CONSTEXPR
    const T* cbegin() const { return _data; }

    _SPL_CONSTEXPR
    const T* cend() const { return _data + _size; }

    _SPL_CONSTEXPR
    const T* begin() const { return _data; }

    _SPL_CONSTEXPR
    const T* end() const { return _data + _size; }

    _SPL_CONSTEXPR
    T* begin() { return _data; }

    _SPL_CONSTEXPR
    T* end() { return _data + _size; }

    _SPL_CONSTEXPR
    const T& operator[] (size_type i) const { return _data[i]; }

    _SPL_CONSTEXPR
    T& operator[] (size_type i) { return _data[i]; }

    _SPL_CONSTEXPR
    const T* data() const noexcept { return _data; }

    _SPL_CONSTEXPR
    T* data() noexcept { return _data; }

    _SPL_CONSTEXPR
    size_type size() const noexcept { return _size; }

    _SPL_CONSTEXPR
    size_type size_bytes() const noexcept { return _size * sizeof (T); }

protected:
    DataT _data {};
    size_type _size {0};
};

} // namespace detail

template<typename T, size_t extent>
class FixedArray : public detail::ArrayBase<T,uint32_t,T[extent]>
{
public:
    using value_type = T;
    using size_type = uint32_t;

    _SPL_CONSTEXPR
    FixedArray() = default;

    static _SPL_CONSTEXPR
    size_type capacity() { return (size_type) extent; }

    _SPL_CONSTEXPR
    void resize(size_type n)
    {
        if (n > this->_size)
        {
            assert(n <= capacity());
            while (this->_size < n)
                detail::construct_at(this->_data + (this->_size++));
        }
        else if (n < this->_size)
        {
            detail::destroy(this->_data+n, this->_data+this->_size);
            this->_size = n;
        }
    }

    template<typename... Args>
    _SPL_CONSTEXPR
    T& emplace_back(Args&&... args)
    {
        assert(this->_size+1 <= capacity());
        detail::construct_at(this->_data + (this->_size++), std::forward<Args>(args)...);
        return this->back();
    }

    _SPL_CONSTEXPR
    void push_back(const T& t)
    {
        assert(this->_size+1 <= capacity());
        detail::construct_at(this->_data + (this->_size++), t);
    }

    _SPL_CONSTEXPR
    void pop_back()
    {
        detail::destroy_at(this->_data + (--this->_size));
    }
};

/**
 * @brief An array that does not own its memory
 */
template<typename T, typename SizeT = int>
class ArrayAdaptor : public detail::ArrayBase<T,SizeT>
{
public:
    using base_type = detail::ArrayBase<T,SizeT>;
    using value_type = T;
    using size_type = SizeT;

    _SPL_CONSTEXPR
    ArrayAdaptor() = default;

    _SPL_CONSTEXPR
    ArrayAdaptor(T* p, size_type max_size)
        : _capacity{max_size} { this->_data = p, this->_size = 0; }

    _SPL_CONSTEXPR
    ArrayAdaptor(T* p, size_type n, size_type max_size)
        : _capacity{max_size} { this->_data = p, this->_size = n; }

    _SPL_CONSTEXPR
    size_type capacity() const
    {
        return _capacity;
    }

    _SPL_CONSTEXPR
    void resize(size_type n)
    {
        if (n > this->_size)
        {
            assert(n <= _capacity);
            while (this->_size < n)
                new (this->_data + (this->_size++)) T();
        }
        else if (n < this->_size)
        {
            detail::destroy(this->_data+n, this->_data+this->_size);
            this->_size = n;
        }
    }

    template<typename... Args>
    _SPL_CONSTEXPR
    T& emplace_back(Args&&... args)
    {
        assert(this->_size+1 <= _capacity);
        new (this->_data + (this->_size++)) T(std::forward<Args>(args)...);
        return this->back();
    }

    _SPL_CONSTEXPR
    void push_back(const T& t)
    {
        assert(this->_size+1 <= _capacity);
        new (this->_data + (this->_size++)) T(t);
    }

    _SPL_CONSTEXPR
    void pop_back()
    {
        detail::destroy_at(this->_data + (--this->_size));
    }

protected:
    size_type _capacity {0};
};

} // namespace cg
