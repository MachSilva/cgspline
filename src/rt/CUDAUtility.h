#pragma once

#include <core/Globals.h>
#include <memory>
#include <memory_resource>

#ifndef NDEBUG
#define CUDA_CHECK(e) \
    ::cg::rt::cudaErrorCheck(cudaGetLastError(),__FILE__,__LINE__), \
    ::cg::rt::cudaErrorCheck(e,__FILE__,__LINE__)
#else
#define CUDA_CHECK(e) \
    ::cg::rt::cudaErrorCheck(cudaGetLastError(),__func__,__LINE__), \
    ::cg::rt::cudaErrorCheck(e,__func__,__LINE__)
#endif

namespace cg::rt
{

void cudaErrorCheck(cudaError_t e, const char* source, int line);

class ManagedResource : public std::pmr::memory_resource
{
public:
    static ManagedResource* instance();

private:
    void* do_allocate(std::size_t n, std::size_t alignment) override;
    void do_deallocate(void* p, std::size_t n, std::size_t alignment) override;
    bool do_is_equal(const std::pmr::memory_resource& r) const noexcept override;
};

template<typename T>
struct async_allocator
{
    using value_type = T;
    using size_type = size_t;

    // You are free to change this at any time
    cudaStream_t stream = 0;

    async_allocator() = default;
    async_allocator(cudaStream_t s) : stream{s} {}

    [[nodiscard]] T* allocate(size_type n)
    {
        T* p;
        CUDA_CHECK(cudaMallocAsync(&p, n * sizeof (T), stream));
        return p;
    }

    void deallocate(T* p, size_type)
    {
        cudaFreeAsync((void*) p, stream);
    }
};

template<typename T>
struct managed_allocator
{
    using value_type = T;
    using size_type = size_t;

    // Flags to cudaMallocManaged
    // uint32_t flags = 1U;

    [[nodiscard]] T* allocate(size_type n)
    {
        T* p;
        CUDA_CHECK(cudaMallocManaged(&p, n * sizeof (T)));
        return p;
    }

    void deallocate(T* p, size_type)
    {
        cudaFree((void*) p);
    }
};

template<typename T1, typename T2>
constexpr
bool operator== (const managed_allocator<T1>&, const managed_allocator<T2>&)
{
    return true;
}

template<typename T1, typename T2>
constexpr
bool operator== (const managed_allocator<T1>&, const async_allocator<T2>&)
{
    return true;
}

template<typename T1, typename T2>
constexpr
bool operator== (const async_allocator<T1>&, const managed_allocator<T2>&)
{
    return true;
}

template<typename T1, typename T2>
constexpr
bool operator== (const async_allocator<T1>&, const async_allocator<T2>&)
{
    return true;
}

template<typename T1, typename T2>
bool operator== (const managed_allocator<T1>&,
    const std::pmr::polymorphic_allocator<T2>& a)
{
    return dynamic_cast<ManagedResource*>(a.resource());
}

template<typename T1, typename T2>
bool operator== (const async_allocator<T1>&,
    const std::pmr::polymorphic_allocator<T2>& a)
{
    return dynamic_cast<ManagedResource*>(a.resource());
}


template<typename T1, typename A2>
constexpr
bool operator== (const managed_allocator<T1>&, const A2&)
{
    return false;
}

template<typename T1, typename A2>
constexpr
bool operator== (const async_allocator<T1>&, const A2&)
{
    return false;
}

/**
 * @brief A template for passing arguments to a CUDA kernel by value without
 * triggering any constructor or destructor.
 */
template<typename T>
struct Raw
{
    union Data
    {
        __align__(alignof(T)) struct {} nothing;
        uint8_t data[sizeof(T)];
    } raw_data;

    __host__ __device__
    Raw() = default;

    __host__ __device__
    Raw(const T& value)
    {
        auto p = reinterpret_cast<const Data*>(&value);
        raw_data = *p;
    }

    __host__ __device__
    const T* get() const
    {
        return reinterpret_cast<const T*>(&raw_data);
    }

    __host__ __device__
    T* get()
    {
        return reinterpret_cast<T*>(&raw_data);
    }

    __host__ __device__
    const T* operator-> () const { return get(); }

    __host__ __device__
    T* operator-> () { return get(); }
};

/**
 * @brief Just an uninitialized array of type @a T . It is an alternative
 *        to `std::pmr::vector` since the latter may initialize all the data.
 */
template<typename T,
    template<typename> typename AllocTpl = std::pmr::polymorphic_allocator>
class Buffer
{
public:
    using value_type = T;
    using allocator_type = AllocTpl<T>;

    Buffer() = default;
    Buffer(const allocator_type& allocator)
        : _length{0}, _alloc{allocator} {}
    Buffer(size_t n, const allocator_type& allocator = {})
        : _length{n}, _alloc{allocator}
    {
        _ptr = _alloc.allocate(n);
    }

    ~Buffer()
    {
        if (_ptr)
            _alloc.deallocate(_ptr, _length);
    }

    Buffer(const Buffer& other)
    {
        *this = other;
    }

    Buffer(Buffer&& other)
    {
        *this = std::move(other);
    }

    Buffer& operator= (const Buffer& other)
    {
        if (this == &other)
            return *this;

        if constexpr (std::allocator_traits<allocator_type>
            ::propagate_on_container_copy_assignment::value)
        {
            _alloc = std::allocator_traits<allocator_type>
                ::select_on_container_copy_construction(other._alloc);
        }

        resize(other.size());
        std::uninitialized_copy_n(other.data(), other.size(), _ptr);

        return *this;
    }

    Buffer& operator= (Buffer&& other)
    {
        if (this == &other)
            return *this;

        if (_alloc != other._alloc)
        {
            throw std::runtime_error(
                "Buffer move assignment: allocators are not equal"
                ", operation for this case not implemented yet");
        }

        std::destroy_at(this);

        if constexpr (std::allocator_traits<allocator_type>
            ::propagate_on_container_move_assignment::value)
        {
            _alloc = std::move(_alloc);
        }

        _length = other._length;
        _ptr = other._ptr;
        other._ptr = nullptr;
        other._length = 0;
        return *this;
    }

    const auto& allocator() const { return _alloc; }

    auto& allocator() { return _alloc; }

    // Release ownership of the data
    T* release() { auto p = _ptr; _ptr = nullptr, _length = 0; return p; }

    /**
     * @brief Resize the buffer. No op. if the requested size is shorter.
     * 
     * @note The previous data will be lost if a larger size is requested.
     */
    void resize(size_t n)
    {
        if (n > _length)
        {
            if (_ptr)
            {
                _alloc.deallocate(_ptr, _length);
            }
            _ptr = _alloc.allocate(_length = n);
        }
    }

    __host__ __device__
    const T* cbegin() const { return _ptr; }

    __host__ __device__
    const T* cend() const { return _ptr + _length; }

    __host__ __device__
    T* begin() { return _ptr; }

    __host__ __device__
    T* end() { return _ptr + _length; }

    __host__ __device__
    const T& operator[] (size_t i) const { return _ptr[i]; }

    __host__ __device__
    T& operator[] (size_t i) { return _ptr[i]; }

    __host__ __device__
    const T* data() const noexcept { return _ptr; }

    __host__ __device__
    T* data() noexcept { return _ptr; }

    __host__ __device__
    size_t size() const noexcept { return _length; }

    __host__ __device__
    size_t size_bytes() const noexcept { return _length * sizeof (T); }

protected:
    allocator_type _alloc {};
    size_t _length {0};
    T* _ptr {nullptr};
};

/**
 * @brief An alternative to `std::vector` since its methods are not marked with
 *        `__host__ __device__` attributes; and an alternative to
 *        `thrust::universal_vector` since it does not require to be compiled
 *        with nvcc and does not launch any kernel.
 */
template<typename T,
    template<typename> typename AllocTpl = std::pmr::polymorphic_allocator>
class Array
{
public:
    using value_type = T;
    using allocator_type = AllocTpl<T>;

    Array() = default;
    Array(const allocator_type& allocator)
        : _alloc{allocator} {}
    Array(size_t n, const allocator_type& allocator = {})
        : _capacity{n}, _size{n}, _alloc{allocator}
    {
        _ptr = _alloc.allocate(n);
        std::uninitialized_default_construct_n(_ptr, n);
    }

    ~Array()
    {
        clear();
        if (_ptr)
            _alloc.deallocate(_ptr, _capacity);
    }

    Array(const Array& other)
    {
        *this = other;
    }

    Array(Array&& other)
    {
        *this = std::move(other);
    }

    Array& operator= (const Array& other)
    {
        if (this == &other)
            return *this;

        if constexpr (std::allocator_traits<allocator_type>
            ::propagate_on_container_copy_assignment::value)
        {
            _alloc = std::allocator_traits<allocator_type>
                ::select_on_container_copy_construction(other._alloc);
        }

        clear();
        reserve(other.size());
        std::uninitialized_copy_n(other.data(), other.size(), _ptr);

        return *this;
    }

    Array& operator= (Array&& other)
    {
        if (this == &other)
            return *this;

        if (_alloc != other._alloc)
        {
            throw std::runtime_error(
                "Array move assignment: allocators are not equal"
                ", operation for this case not implemented yet");
        }

        std::destroy_at(this);

        if constexpr (std::allocator_traits<allocator_type>
            ::propagate_on_container_move_assignment::value)
        {
            _alloc = std::move(_alloc);
        }

        _capacity = other._capacity;
        _size = other._size;
        _ptr = other._ptr;
        other._ptr = nullptr;
        other._size = 0;
        other._capacity = 0;
        return *this;
    }

    const auto& allocator() const { return _alloc; }

    auto& allocator() { return _alloc; }

    void clear()
    {
        std::destroy_n(_ptr, _size);
        _size = 0;
    }

    // Release ownership of the data
    T* release() noexcept
    {
        auto p = _ptr;
        _ptr = nullptr, _size = 0, _capacity = 0;
        return p;
    }

    void reserve(size_t n)
    {
        if (n > _capacity)
        {
            T* p = _alloc.allocate(n);
            if (_ptr)
            {
                std::uninitialized_move_n(_ptr, _size, p);
                _alloc.deallocate(_ptr, _capacity);
            }
            _ptr = p;
            _capacity = n;
        }
    }

    void resize(size_t n)
    {
        if (n > _size)
        {
            reserve(n);
            std::uninitialized_default_construct(_ptr+_size, _ptr+n);
            _size = n;
        }
        else if (n < _size)
        {
            std::destroy(_ptr+n, _ptr+_size);
            _size = n;
        }
    }

    template<typename... Args>
    T& emplace_back(Args&&... args)
    {
        grow();
        ::new (static_cast<void*>(_ptr + (_size++))) T(std::forward<Args>(args)...);
        return back();
    }

    void push_back(const T& t)
    {
        grow();
        ::new (static_cast<void*>(_ptr + (_size++))) T(t);
    }

    void pop_back()
    {
        std::destroy_at(_ptr + (--_size));
    }

    __host__ __device__
    const T& front() const { return *_ptr; }

    __host__ __device__
    const T& back() const { return _ptr[_size - 1]; }

    __host__ __device__
    T& front() { return *_ptr; }

    __host__ __device__
    T& back() { return _ptr[_size - 1]; }

    __host__ __device__
    const T* cbegin() const { return _ptr; }

    __host__ __device__
    const T* cend() const { return _ptr + _size; }

    __host__ __device__
    const T* begin() const { return _ptr; }

    __host__ __device__
    const T* end() const { return _ptr + _size; }

    __host__ __device__
    T* begin() { return _ptr; }

    __host__ __device__
    T* end() { return _ptr + _size; }

    __host__ __device__
    const T& operator[] (size_t i) const { return _ptr[i]; }

    __host__ __device__
    T& operator[] (size_t i) { return _ptr[i]; }

    __host__ __device__
    const T* data() const noexcept { return _ptr; }

    __host__ __device__
    T* data() noexcept { return _ptr; }

    __host__ __device__
    size_t capacity() const noexcept { return _capacity; }

    __host__ __device__
    size_t size() const noexcept { return _size; }

    __host__ __device__
    size_t size_bytes() const noexcept { return _size * sizeof (T); }

protected:
    allocator_type _alloc {};
    size_t _capacity {0};
    size_t _size {0};
    T* _ptr {nullptr};

    void grow()
    {
        if (_size >= _capacity)
        {
            reserve(_capacity < 2
                ? 2
                : _capacity + (_capacity >> 1)
            );
        }
    }
};

template<typename C>
requires requires (C c)
{
    typename C::value_type;
    { c.size() } -> std::convertible_to<size_t>;
}
size_t size_bytes(const C& c) noexcept
{
    return c.size() * sizeof (typename C::value_type);
}

template<typename T> using ManagedBuffer = Buffer<T,managed_allocator>;
template<typename T> using ManagedArray = Array<T,managed_allocator>;

} // namespace cg::rt
