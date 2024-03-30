#pragma once

#include <core/Globals.h>
#include <memory>
#include <memory_resource>

#ifndef NDEBUG
#define CUDA_CHECK(e) ::cg::rt::cudaErrorCheck(e, __FILE__, __LINE__)
#else
#define CUDA_CHECK(e) ::cg::rt::cudaErrorCheck(e, __func__, __LINE__)
#endif

namespace cg::rt
{

void cudaErrorCheck(cudaError_t e, const char* source, int line);

template<typename T>
class ManagedAllocator
{
public:
    static T* allocate(size_t n)
    {
        T* ptr;
        CUDA_CHECK(cudaMallocManaged(&ptr, n * sizeof (T)));
        return ptr;
    }

    static void free(T* ptr)
    {
        CUDA_CHECK(cudaFree(ptr));
    }
};

class ManagedCUDAResource : public std::pmr::memory_resource
{
public:
    static ManagedCUDAResource* get_instance();

private:
    void* do_allocate(std::size_t n, std::size_t alignment) override;
    void do_deallocate(void* p, std::size_t n, std::size_t alignment) override;
    bool do_is_equal(const std::pmr::memory_resource& r) const noexcept override;
};

// template<typename T> using ManagedArray = cg::ArrayBase<T,ManagedAllocator<T>>;

/**
 * @brief A template for passing arguments to a CUDA kernel by value without
 * triggering its constructors or destructor.
 */
template<typename T>
struct KernelArg
{
    union RawData
    {
        __align__(alignof(T)) struct {} nothing;
        uint8_t data[sizeof(T)];
    } raw_data;

    __host__ __device__
    KernelArg(const T& value)
    {
        auto p = reinterpret_cast<const RawData*>(value);
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
};

/**
 * @brief Just an uninitialized array of type @a T . It is an alternative
 *        to `std::pmr::vector` since it does initialize all the data.
 */
template<typename T, typename Alloc = std::pmr::polymorphic_allocator<T>>
class Buffer
{
public:
    using value_type = T;
    using allocator_type = Alloc;

    Buffer() = default;
    Buffer(size_t n, const Alloc& allocator = {})
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

    const T& operator[] (size_t i) const { return _ptr[i]; }
    T& operator[] (size_t i) { return _ptr[i]; }

    const T* data() const noexcept { return _ptr; }
    T* data() noexcept { return _ptr; }

    size_t size() const noexcept { return _length; }
    size_t size_bytes() const noexcept { return _length * sizeof (T); }

protected:
    Alloc _alloc {};
    size_t _length {0};
    T* _ptr {nullptr};
};

} // namespace cg::rt
