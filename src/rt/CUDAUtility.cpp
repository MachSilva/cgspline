#include "CUDAUtility.h"

#include <iostream>

namespace cg::rt
{

void cudaErrorCheck(cudaError_t e, const char* source, int line)
{
    if (e != cudaSuccess)
    {
        // auto e2 = cudaDeviceReset();
        fprintf(stderr, "CUDA error '%s' (%d) at %s:%d: %s\n",
            cudaGetErrorName(e),
            e,
            source,
            line,
            cudaGetErrorString(e)
        );
        // if (e2 != cudaSuccess)
        //     fprintf(stderr, "Failed to reset device: %s (%d)\n",
        //         cudaGetErrorString(e2), e2);
        throw std::runtime_error("CUDA Error");
    }
}

ManagedResource* ManagedResource::get_instance()
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
    return ptr;
}

void ManagedResource::do_deallocate(void* p, std::size_t n, std::size_t alignment)
{
    cudaFree(p);
}

bool ManagedResource::do_is_equal(const std::pmr::memory_resource& r) const noexcept
{
    return dynamic_cast<const ManagedResource*>(&r) != nullptr;
}

} // namespace cg::rt
