#pragma once

#include <core/Globals.h>
#include <memory>
#include <memory_resource>
#include <thrust/device_allocator.h>

#ifndef NDEBUG
#define CUDA_CHECK(e) ::cg::rt::cudaErrorCheck(e, __FILE__, __LINE__)
#elif defined(__FUNCSIG__)
#define CUDA_CHECK(e) ::cg::rt::cudaErrorCheck(e, __FUNCSIG__, __LINE__)
#elif defined(__GNUC__)
#define CUDA_CHECK(e) ::cg::rt::cudaErrorCheck(e, __FUNCTION__, __LINE__)
#else
#define CUDA_CHECK(e) ::cg::rt::cudaErrorCheck(e, __func__, __LINE__)
#endif

namespace cg::rt
{

void cudaErrorCheck(cudaError_t e, const char* source, int line);

class ManagedCUDAResource : public std::pmr::memory_resource
{
public:
    static ManagedCUDAResource* get_instance();

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

/**
 * @brief A template for passing arguments to a CUDA kernel by value without
 * triggering any constructor or destructor.
 */
template<typename T>
struct Raw
{
    union RawData
    {
        __align__(alignof(T)) struct {} nothing;
        uint8_t data[sizeof(T)];
    } raw_data;

    __host__ __device__
    Raw() = default;

    __host__ __device__
    Raw(const T& value)
    {
        auto p = reinterpret_cast<const RawData*>(&value);
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

    Buffer(const Buffer& other) = delete;
    Buffer(Buffer&& other)
    {
        *this = std::move(other);
    }

    Buffer& operator= (const Buffer& other) = delete;
    Buffer& operator= (Buffer&& other)
    {
        if (this == &other)
            return *this;
        std::swap(_alloc, other._alloc);
        std::swap(_length, other._length);
        std::swap(_ptr, other._ptr);
        return *this;
    }

    const auto& allocator() const { return _alloc; }

    auto& allocator() { return _alloc; }

    // Release ownership of the data
    T* release() { auto p = _ptr; _ptr = nullptr; return p; }

    /**
     * @brief Resize the buffer. No op. if the requested size is shorter.
     * 
     * @note The previous data will be lost if a larger size is requested.
     */
    void resize(size_t n)
    {
        if (n > _length)
        {
            _alloc.deallocate(_ptr, _length);
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

} // namespace cg::rt
