#include "CUDAUtility.h"

#include "../Log.h"

namespace cg::rt
{

void cudaErrorCheck(cudaError_t e, const char* source, int line)
{
    if (e != cudaSuccess)
    {
        log::error(
            "CUDA: '{}' ({}) at {}:{}: {}",
            cudaGetErrorName(e),
            (int) e,
            source,
            line,
            cudaGetErrorString(e)
        );
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
    // log::debug("ManagedResource: allocate {:8} bytes at {} - {}", n, ptr,
    //     (void*)(((uint8_t*)ptr)+n));
    return ptr;
}

void ManagedResource::do_deallocate(void* p, std::size_t n, std::size_t alignment)
{
    // log::debug("ManagedResource: free     {:8} bytes at {}", n, p);
    cudaFree(p);
}

bool ManagedResource::do_is_equal(const std::pmr::memory_resource& r) const noexcept
{
    return dynamic_cast<const ManagedResource*>(&r) != nullptr;
}

} // namespace cg::rt
