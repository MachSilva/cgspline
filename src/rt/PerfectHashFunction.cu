#include "PerfectHashFunction.h"

// #define CUB_DEBUG_SYNC

#include <assert.h>
#include <cub/cub.cuh>
#include <unordered_set>
#include "ScopeClock.h"
#include "../Log.h"

namespace cg::rt
{

constexpr uint32_t pigeonholeCount(uint32_t pigeonCount, uint32_t holeCapacity)
{
    return (pigeonCount / holeCapacity) + (pigeonCount % holeCapacity ? 1 : 0);
}

static constexpr unsigned kBucketCapacity = 4U;

/**
 * @param bucketArray is a 2D matrix with @a kBucketCapacity elements per line
 *        and @a bucketCount elements per column. Each line is a contiguous
 *        region in memory. Each line of this matrix is referred as a @e bucket.
 */
// template<unsigned kBucketCapacity = 8U>
__global__ __launch_bounds__(32)
void init_and_bucketsort(
    uint32_t* __restrict__ bucketArray,
    uint32_t* __restrict__ bucketSizeArray,
    uint32_t* __restrict__ bucketIdArray,
    uint32_t bucketCount,   // Must be a power of 2
    uint32_t bucketMask,    // = bucketCount - 1
    const uint32_t* __restrict__ keys,
    uint32_t keyCount)
{
    const uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= keyCount)
        return;

    uint32_t j = keys[i] & bucketMask; // displacement index
    uint32_t k = atomicInc(bucketSizeArray + j, INT32_MAX);

    // This is not allowed (an almost impossible occurrence)
    if (k >= kBucketCapacity)
        __trap();

    bucketArray[j * kBucketCapacity + k] = keys[i];

    // ID array init
    if (i < bucketCount)
    {
        bucketIdArray[i] = i;
    }
}

#if 0
/**
 * @note @a result expected to be initialized with zero
 * @note Uses 4 bytes of shared memory
 */
__global__
void feasibility_test(
    uint32_t* __restrict__ result,
    uint32_t hashA,
    uint32_t hashB,
    const uint32_t* __restrict__ bucketArray,
    const uint32_t* __restrict__ bucketSizeArray,
    uint32_t bucketCount)
{
    const uint32_t tId = blockIdx.x * blockDim.x + threadIdx.x;
    if (tId >= bucketCount)
        return;

    uint32_t __shared__ earlyExit;
    if (threadIdx.x == 0)
        earlyExit = 0;
    
    __syncthreads();

    const uint32_t* bucket = bucketArray + (tId * kBucketCapacity);
    const uint32_t n = bucketSizeArray[tId];
    for (int i = 1; i < n; i++)
    {
        const uint32_t k = (hashA * __ldg(bucket + i) + hashB);
        for (int j = 0; j < i; j++)
        {
            if (k == (hashA * __ldg(bucket + j) + hashB) || earlyExit)
            {
                *result = 1;
                earlyExit = 1;
                return;
            }
        }
    }
}
#endif

/**
 * @note @a histogram expected to be initialized with zero
 */
__global__ __launch_bounds__(32)
void dependency_histogram(
    uint32_t* __restrict__ histogram,       // [0, kMaxBuckets + 1)
    const uint32_t* __restrict__ bucketSizeArray,
    uint32_t bucketCount)
{
    const uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= bucketCount)
        return;

    atomicInc(histogram + bucketSizeArray[i], INT32_MAX);
}

/**
 * @brief Sort buckets ids (indices) in descending order
 * 
 * @note @a tmpStorage expected to be initialized with zero
 */
__global__
void sort_buckets_descending(
    uint32_t* __restrict__ bucketIdArray,           // [0, bucketCount)
    uint32_t* __restrict__ tmpStorage,              // [0, kMaxBuckets + 1)
    const uint32_t* __restrict__ prefixedHistogram, // [0, kMaxBuckets + 1)
    const uint32_t* __restrict__ bucketSizeArray,   // [0, bucketCount)
    uint32_t bucketCount
)
{
    const uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= bucketCount)
        return;
    
    auto size = bucketSizeArray[i];
    assert(size <= kBucketCapacity);
    // Write in reverse. (use the inclusive scan)
    auto base = bucketCount - prefixedHistogram[size];

    auto idx = atomicInc(tmpStorage + size, INT32_MAX);
    bucketIdArray[base + idx] = i;
}

/**
 * @attention This kernel must be launched with one thread per an element in a
 *            bucket ( @a kBucketCapacity, @a bucketIdCount )
 */
__global__
void insert_keys(
    int* __restrict__ pFailureCounter,
    uint32_t hashA,
    uint32_t hashB,
    uint32_t* __restrict__ hashTable,
    uint32_t hashTableSize,
    uint32_t* __restrict__ displacementTable,
    const uint32_t* __restrict__ bucketArray,
    uint32_t bucketCount,   // Must be a power of 2
    uint32_t bucketMask,    // = bucketCount - 1
    const uint32_t* __restrict__ bucketIdArray, // Contains indices relative to `bucketArray`
    uint32_t bucketIdCount,
    uint32_t dependencySize,
    const uint32_t* __restrict__ keys,
    uint32_t keyCount)
{
    // threadIdx.x maps elements
    // threadIdx.y maps buckets
    const uint32_t bucketGroup = blockIdx.x * blockDim.y + threadIdx.y;

    if (bucketGroup >= bucketIdCount || threadIdx.x >= dependencySize)
        return;

    const auto bucketId = bucketIdArray[bucketGroup];
    const auto key = bucketArray[bucketId * kBucketCapacity + threadIdx.x];

    assert(blockDim.x == kBucketCapacity);
    assert(warpSize == 32);
    // Find my warp group. Let's split this warp in four groups of eight threads
    // For kBucketCapacity = 8
    // {
    // static_assert(kBucketCapacity == 8);
    // const uint32_t myWarpGroup = bucketGroup % 4;
    // const uint32_t mask = 0xFF << (myWarpGroup * 8);
    // }

    // For kBucketCapacity = 4
    // {
    static_assert(kBucketCapacity == 4);
    const uint32_t myWarpGroup = bucketGroup % 8;
    const uint32_t mask = 0x0F << (myWarpGroup * 4);
    // }

    const auto keyHash = hashA * key + hashB;

    assert(bucketId == (key & bucketMask));

    // Try to fill the hash table slots
    int attempt = 0;
    uint32_t displacement = 0;
    constexpr int kMaxAttempts = 32;
    for (; attempt < kMaxAttempts; attempt++) // Maximum of attempts before failure
    {
        auto hashIdx = (keyHash + displacement) % hashTableSize;
        auto hashPtr = hashTable + hashIdx;
        auto conflict = atomicInc(hashPtr, INT32_MAX);
        if (__any_sync(mask, conflict))
        {
            // Avoid decrementing before incrementing
            __threadfence();
            atomicDec(hashPtr, INT32_MAX); // Undo increment
            displacement += bucketGroup + 1;
        }
        else
        {
            // All slots free!
            if (threadIdx.x == 0)
            {
                // Register found displacement and quit
                displacementTable[bucketId] = displacement;
            }
            break;
        }
    }

    if (threadIdx.x == 0)
    {
        if (attempt >= kMaxAttempts)
            atomicInc((uint32_t*) pFailureCounter, INT32_MAX);
    }
}

int PerfectHashFunction::build(span<const uint32_t> keys, cudaStream_t stream)
{
    // SPL_TIME_THIS();

    const auto n = keys.size();
    int s = int(n - 1) >> 1;

    assert(n < 0xFFFFFFFF &&
        "PerfectHashFunction: table size too big for an unsigned 32-bit integer");

    // Minimum size for the displacement table
    if (s < 2)
    {
        s = 2;
        // log::warn("PerfectHashFunction: key set is too small to need a hash table (|S| = {})", n);
    }

    const auto displSize = std::bit_floor((unsigned) s);
    _displacement.resize(displSize);
    _displMask = displSize - 1U;
    _tableSize = n;

    const uint32_t sizeIncreaseStep = std::max<uint32_t>(4U, 0.05f * n);

    constexpr auto device = 0;
    CUDA_CHECK(cudaMemPrefetchAsync((const void*)_displacement.data(),
        _displacement.size() * sizeof (uint32_t), device, stream));

    // Map all keys to its displacement index.
    // For each displacement index, compute how many keys map to it.
    // Sort the indices by the number of keys that depend on it,
    //      from the index with the largest set of keys to the index with
    //      shortest set of keys.
    // For each set, from the largest to the shortest, hash each key to its
    //      index in the table. If there is any conflict, change the value of
    //      the displacement related to the current key set and try this step
    //      again (fail after many attempts).
    //      Otherwise, place all the keys in the table.

    // Now, a lower level algorithm:
    // Take each displacement index as a bucket.
    // Map all keys into a bucket.
    // Find the maximum number K of keys inside a single bucket.
    // Create a bucket list.
    // For `i` from K down to 1:
    //      Partition the list into a set with buckets of i keys and another set
    //      with remaining buckets.
    //      Take the first set and launch a kernel put their keys in the hash
    //      table.

    Buffer<uint32_t,async_allocator> vkeys (n, stream); 
    CUDA_CHECK(cudaMemcpyAsync(vkeys.data(), keys.data(),
        keys.size_bytes(), cudaMemcpyHostToDevice, stream));

    Buffer<uint32_t,async_allocator> hashTable (n, stream);

    // log::info("PerfectHashFunction: {} displacement buckets; {} keys",
    //     displSize, keys.size());

    Buffer<uint32_t,async_allocator> buckets (kBucketCapacity*displSize, stream);
    Buffer<uint32_t,async_allocator> bucketSizes (displSize, stream);
    Buffer<uint32_t,async_allocator> bucketIdArray (displSize, stream);

    CUDA_CHECK(cudaMemsetAsync(bucketSizes.data(), 0,
        bucketSizes.size_bytes(), stream));
    
    CUDA_CHECK(cudaStreamSynchronize(stream));

    constexpr auto kThreadsPerBlock = 32;
    // const auto blocks = (n / kThreadsPerBlock) + (n % kThreadsPerBlock ? 1 : 0);
    const auto blocks = pigeonholeCount(n, kThreadsPerBlock);
    init_and_bucketsort<<<blocks, kThreadsPerBlock, 0, stream>>>(
        (uint32_t*) buckets.data(),
        bucketSizes.data(),
        bucketIdArray.data(),
        displSize,
        _displMask,
        vkeys.data(),
        vkeys.size());
    
    // Result array (managed since it is small and reads will be frequent)
    Buffer<uint32_t,managed_allocator> result (4);
    CUDA_CHECK(cudaMemPrefetchAsync((const void*)result.data(),
        result.size_bytes(), device, stream));

    Buffer<uint8_t,async_allocator> tmpStorage (0x100, stream);
    size_t tmpStorageSize;

    const auto tmpStorageSizeCheck = [&]()
    {
        if (tmpStorageSize > tmpStorage.size_bytes())
        {
            // log::debug(
            //     "\ttemporary storage buffer increased from {}"
            //     " bytes to (requested) {} bytes",
            //     tmpStorage.size_bytes(), tmpStorageSize);
            tmpStorage.resize(tmpStorageSize);
        }
    };
    
    Buffer<uint32_t,managed_allocator> histogram (kBucketCapacity + 1);
    CUDA_CHECK(cudaMemsetAsync(histogram.data(), 0,
        histogram.size_bytes(), stream));

    CUDA_CHECK(cudaMemsetAsync(tmpStorage.data(), 0,
        (kBucketCapacity + 1) * sizeof (uint32_t), stream));

    const auto displGrid = pigeonholeCount(displSize, kThreadsPerBlock);
    dependency_histogram<<<displGrid,kThreadsPerBlock,0,stream>>>(
        histogram.data(), bucketSizes.data(), displSize
    );

    CUDA_CHECK(cudaStreamSynchronize(stream));

    // Find maximum
    int bucketGreatestSize = 0;
    for (int i = 0; i < histogram.size(); i++)
    {
        if (histogram[i] > 0)
            bucketGreatestSize = i;
    }

    CUDA_CHECK(cudaMemPrefetchAsync((const void*) histogram.data(),
        histogram.size_bytes(), device, stream));

    // XXX Parallel inclusive sum is unnecessary with such small vector;
    // TODO: remove
    // Sort buckets
    // 1. compute prefix sum
    CUDA_CHECK(cub::DeviceScan::InclusiveSum(nullptr, tmpStorageSize,
        histogram.data(), histogram.size(), stream));
    
    tmpStorageSizeCheck();

    CUDA_CHECK(cub::DeviceScan::InclusiveSum(tmpStorage.data(), tmpStorageSize,
        histogram.data(), histogram.size(), stream));

    CUDA_CHECK(cudaStreamSynchronize(stream));

    // 2. sort
    CUDA_CHECK(cudaMemsetAsync(tmpStorage.data(), 0,
        (kBucketCapacity + 1) * sizeof (uint32_t), stream));

    sort_buckets_descending<<<displGrid,kThreadsPerBlock,0,stream>>>(
        bucketIdArray.data(),
        (uint32_t*) tmpStorage.data(),
        histogram.data(),
        bucketSizes.data(),
        displSize
    );
    std::uniform_int_distribution<uint32_t> d (0, n);

    int hashTableSize = keys.size();

    for (int hashId = 0; hashId < 16; hashId++)
    {
        uint32_t cofactor;
        int j = 0;
        do
        {
            _A = d(_mt);
            if (_A == 0)
                _A = 31; // _A cannot be zero
            _B = d(_mt);
            cofactor = std::gcd(keys.size(), _A);

            // log::info("\ttest hash(k) = {} * k + {} mod N; "
            //     "gcd(N, h.A) = {}",
            //     _A, _B,
            //     cofactor);
        }
        while (cofactor > 1 && j < 16); // make it safe against infinite loops

#if 0
        result[0] = 0;

        // Test hash function feasibility
        feasibility_test<<<blocks,kThreadsPerBlock,4,stream>>>(
            result.data(), _A, _B, buckets.data(), bucketSizes.data(), displSize
        );

        CUDA_CHECK(cudaStreamSynchronize(stream));

        if (result[0] != 0)
        {
            std::error("\tSKIPPING... impossible hash function");
            continue;
        }
#endif

        CUDA_CHECK(cudaMemsetAsync((void*) result.data(), 0,
            result.size_bytes(), stream));
        
        CUDA_CHECK(cudaMemPrefetchAsync(result.data(), result.size_bytes(),
            device, stream));
        
        CUDA_CHECK(cudaMemsetAsync((void*) _displacement.data(), 0,
            _displacement.size() * sizeof (uint32_t), stream));

        CUDA_CHECK(cudaMemsetAsync(hashTable.data(), 0,
            hashTable.size_bytes(), stream)); // Set hash table values to zero

        // Partition loop
        uint32_t* pIn = bucketIdArray.data();
        uint32_t partitionSize = displSize;
        int selectedCount;

        auto dependencyCount = bucketGreatestSize;
        while (dependencyCount > 0 && partitionSize > 0)
        {
            auto dependencyPrefix = histogram[dependencyCount];
            auto prevDependencyPrefix = histogram[dependencyCount - 1];
            auto offset = displSize - dependencyPrefix;
            selectedCount = dependencyPrefix - prevDependencyPrefix;

            if (selectedCount > 0)
            {
                auto grid = pigeonholeCount(selectedCount, kThreadsPerBlock);
                insert_keys<<<grid, dim3(kBucketCapacity, kThreadsPerBlock), 0, stream>>>(
                    (int*) result.data(),
                    _A,
                    _B,
                    hashTable.data(),
                    hashTable.size(),
                    _displacement.data(),
                    (uint32_t*) buckets.data(),
                    displSize,
                    _displMask,
                    pIn + offset,
                    selectedCount,
                    dependencyCount,
                    keys.data(),
                    keys.size()
                );
            }

            --dependencyCount;
        }

        CUDA_CHECK(cudaStreamSynchronize(stream));

        // log::debug("\t\tFailure count: {}", (int) result[0]);

        // Success?
        if (result[0] == 0)
        {
            // log::debug("\t\tSuccess with hash function number {}", hashId);
            break;
        }

        // In case of failure
        // Increase table size
        _tableSize = hashTable.size() + sizeIncreaseStep;
        hashTable.resize(_tableSize);
        // log::debug("\tN <- {}", _tableSize);
    }

    std::unordered_set<uint32_t> hashSet {};
    bool duplicate = false;
    for (auto& key : keys)
    {
        auto v = hash(key);
        duplicate |= hashSet.contains(v);
        hashSet.emplace(v);
    }

    if (duplicate)
        log::error("PerfectHashFunction: DUPLICATE FOUND!");
    // else
    //     log::info("\tNo duplicate. Perfect Hash! "
    //         "hash table: {} bytes, displacement table: {} bytes",
    //         _tableSize * sizeof (uint32_t), _displacement.size_bytes());

    return (int) duplicate;
}

} // namespace cg::rt
