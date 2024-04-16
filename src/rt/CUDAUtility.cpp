#include "CUDAUtility.h"

#include <format>
#include <iostream>

namespace cg::rt
{

void cudaErrorCheck(cudaError_t e, const char* source, int line)
{
    if (e != cudaSuccess)
    {
        std::string msg = std::format(
            "CUDA: '{}' ({}) at {}:{}: {}\n",
            cudaGetErrorName(e),
            (int) e,
            source,
            line,
            cudaGetErrorString(e)
        );
        std::cerr << msg << std::endl;
        abort();
    }
}

ManagedResource* ManagedResource::instance()
{
    static ManagedResource instance {};
    return &instance;
}

void* ManagedResource::do_allocate(std::size_t n, std::size_t alignment)
{
    void* ptr;
    if (alignment > 256)
        throw std::runtime_error("memory alignment not supported");
    CUDA_CHECK(cudaMallocManaged(&ptr, n));
    fprintf(stderr, "ManagedResource: allocate %8zu bytes at %p - %p\n", n, ptr, ((uint8_t*)ptr)+n);
    return ptr;
}

void ManagedResource::do_deallocate(void* p, std::size_t n, std::size_t alignment)
{
    fprintf(stderr, "ManagedResource: free     %8zu bytes at %p\n", n, p);
    cudaFree(p);
}

bool ManagedResource::do_is_equal(const std::pmr::memory_resource& r) const noexcept
{
    return dynamic_cast<const ManagedResource*>(&r) != nullptr;
}

} // namespace cg::rt
