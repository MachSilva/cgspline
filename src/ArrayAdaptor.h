#pragma once

#include <cuda/std/array>
#include <cuda/std/cassert>
#include <cuda/std/span>

#define _SPL_CONSTEXPR_ATTR __host__ __device__ constexpr

namespace cg
{

namespace custd = ::cuda::std;

namespace detail
{

template<typename T, typename SizeT, typename DataT = T*>
class ArrayBase
{
public:
    using value_type = T;
    using size_type = SizeT;

    _SPL_CONSTEXPR_ATTR
    void clear()
    {
        custd::destroy_n(_data, _size);
        _size = 0;
    }

    _SPL_CONSTEXPR_ATTR
    bool empty() const { return _size == 0; }

    _SPL_CONSTEXPR_ATTR
    const T& front() const { return _data[0]; }

    _SPL_CONSTEXPR_ATTR
    const T& back() const { return _data[_size - 1]; }

    _SPL_CONSTEXPR_ATTR
    T& front() { return _data[0]; }

    _SPL_CONSTEXPR_ATTR
    T& back() { return _data[_size - 1]; }

    _SPL_CONSTEXPR_ATTR
    const T* cbegin() const { return _data; }

    _SPL_CONSTEXPR_ATTR
    const T* cend() const { return _data + _size; }

    _SPL_CONSTEXPR_ATTR
    const T* begin() const { return _data; }

    _SPL_CONSTEXPR_ATTR
    const T* end() const { return _data + _size; }

    _SPL_CONSTEXPR_ATTR
    T* begin() { return _data; }

    _SPL_CONSTEXPR_ATTR
    T* end() { return _data + _size; }

    _SPL_CONSTEXPR_ATTR
    const T& operator[] (size_type i) const { return _data[i]; }

    _SPL_CONSTEXPR_ATTR
    T& operator[] (size_type i) { return _data[i]; }

    _SPL_CONSTEXPR_ATTR
    const T* data() const noexcept { return _data; }

    _SPL_CONSTEXPR_ATTR
    T* data() noexcept { return _data; }

    _SPL_CONSTEXPR_ATTR
    size_type size() const noexcept { return _size; }

    _SPL_CONSTEXPR_ATTR
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

    _SPL_CONSTEXPR_ATTR
    FixedArray() = default;

    static _SPL_CONSTEXPR_ATTR
    size_type capacity() { return (size_type) extent; }

    _SPL_CONSTEXPR_ATTR
    void resize(size_type n)
    {
        if (n > this->_size)
        {
            assert(n <= capacity());
            while (this->_size < n)
                custd::construct_at(this->_data + (this->_size++));
        }
        else if (n < this->_size)
        {
            custd::destroy(this->_data+n, this->_data+this->_size);
            this->_size = n;
        }
    }

    template<typename... Args>
    _SPL_CONSTEXPR_ATTR
    T& emplace_back(Args&&... args)
    {
        assert(this->_size+1 <= capacity());
        custd::construct_at(this->_data + (this->_size++), custd::forward<Args>(args)...);
        return this->back();
    }

    _SPL_CONSTEXPR_ATTR
    void push_back(const T& t)
    {
        assert(this->_size+1 <= capacity());
        custd::construct_at(this->_data + (this->_size++), t);
    }

    _SPL_CONSTEXPR_ATTR
    void pop_back()
    {
        custd::destroy_at(this->_data + (--this->_size));
    }
};

/**
 * @brief An array that does not own its memory
 */
template<typename T, typename SizeT = uint32_t>
class ArrayAdaptor : public detail::ArrayBase<T,SizeT>
{
public:
    using base_type = detail::ArrayBase<T,SizeT>;
    using value_type = T;
    using size_type = SizeT;

    _SPL_CONSTEXPR_ATTR
    ArrayAdaptor() = default;

    _SPL_CONSTEXPR_ATTR
    ArrayAdaptor(T* p, size_type max_size)
        : base_type{ ._data = p, ._size = 0}, _capacity{max_size} {}

    _SPL_CONSTEXPR_ATTR
    ArrayAdaptor(T* p, size_type n, size_type max_size)
        : base_type{ ._data = p, ._size = n}, _capacity{max_size} {}

    _SPL_CONSTEXPR_ATTR
    size_type capacity() const
    {
        return _capacity;
    }

    _SPL_CONSTEXPR_ATTR
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
            custd::destroy(this->_data+n, this->_data+this->_size);
            this->_size = n;
        }
    }

    template<typename... Args>
    _SPL_CONSTEXPR_ATTR
    T& emplace_back(Args&&... args)
    {
        assert(this->_size+1 <= _capacity);
        new (this->_data + (this->_size++)) T(custd::forward<Args>(args)...);
        return this->back();
    }

    _SPL_CONSTEXPR_ATTR
    void push_back(const T& t)
    {
        assert(this->_size+1 <= _capacity);
        new (this->_data + (this->_size++)) T(t);
    }

    _SPL_CONSTEXPR_ATTR
    void pop_back()
    {
        custd::destroy_at(this->_data + (--this->_size));
    }

protected:
    size_type _capacity {0};
};

} // namespace cg
