#include "PerfectHashTable.h"

// #define CUB_DEBUG_SYNC

#include <assert.h>
#include <cub/cub.cuh>
#include <unordered_set>
#include "ScopeClock.h"

namespace cg::rt
{

using namespace thrust::placeholders;

template<typename T>
struct bitmask_fn
{
    T mask;

    __host__ __device__
    constexpr T operator() (T a) const
    {
        return a & mask;
    }
};

template<typename T>
struct index_select_fn
{
    const T* lookup;
    T value;

    __host__ __device__
    constexpr bool operator() (T a) const
    {
        return lookup[a] == value;
    }
};

struct conflict_any_fn
{
    const HashFunction h;
    const uint32_t mask;

    __host__ __device__
    constexpr bool operator() (uint32_t a, uint32_t b) const
    {
        return h(a) == h(b) && (a & mask) == (b & mask);
    }
};

static constexpr unsigned kBucketMaxSize = 8U;

/**
 * @param bucketArray is a 2D matrix with @a kBucketMaxSize elements per line
 *        and @a bucketCount elements per column. Each line is a contiguous
 *        region in memory. Each line of this matrix is referred as a @e bucket.
 */
// template<unsigned kBucketMaxSize = 8U>
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
    if (k >= kBucketMaxSize)
        __trap();

    bucketArray[j * kBucketMaxSize + k] = keys[i];

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
    HashFunction h,
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

    const uint32_t* bucket = bucketArray + (tId * kBucketMaxSize);
    const uint32_t n = bucketSizeArray[tId];
    for (int i = 1; i < n; i++)
    {
        const uint32_t k = h(__ldg(bucket + i));
        for (int j = 0; j < i; j++)
        {
            if (k == h(__ldg(bucket + j)) || earlyExit)
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
    assert(size <= kBucketMaxSize);
    // Write in reverse. (use the inclusive scan)
    auto base = bucketCount - prefixedHistogram[size];

    auto idx = atomicInc(tmpStorage + size, INT32_MAX);
    bucketIdArray[base + idx] = i;
}

/**
 * @attention This kernel must be launched with one thread per an element in a
 *            bucket ( @a kBucketMaxSize, @a bucketIdCount )
 */
__global__
void insert_keys(
    int* __restrict__ pFailureCounter,
    HashFunction h,
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
    const auto key = bucketArray[bucketId * kBucketMaxSize + threadIdx.x];

    // This code expects kBucketMaxSize = 8 and blockDim.x = 8, always!
    // And it also expects warpSize = 32
    static_assert(kBucketMaxSize == 8);
    assert(blockDim.x == 8);
    assert(warpSize == 32);
    // Find my warp group. Let's split this warp in four groups of eight threads
    const uint32_t myWarpGroup = bucketGroup % 4;
    const uint32_t mask = 0xFF << (myWarpGroup * 8);

    const auto keyHash = h(key);

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

int PerfectHashTable::build(span<const uint32_t> keys, cudaStream_t stream)
{
    SPL_TIME_THIS();

    const auto n = keys.size();
    uint32_t s = uint32_t(n >> 1U) - 1U;

    assert(n < 0xFFFFFFFF &&
        "PerfectHashTable: table size too big for an unsigned 32-bit integer");
    assert(s < n && // bit flip
        "PerfectHashTable: key set is too small to need a hash table; "
        "displacement array size will be zero and its behaviour undefined");

    const auto displSize = std::max(2U, std::bit_floor(s));
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

    fprintf(stderr,
        "PerfectHashTable:\n"
        "\t%u displacement buckets\n"
        "\t%u keys\n",
        displSize, keys.size());

    constexpr uint32_t bucketCapacity = 8;
    Buffer<uint32_t,async_allocator> buckets (bucketCapacity*displSize, stream);
    Buffer<uint32_t,async_allocator> bucketSizes (displSize, stream);
    Buffer<uint32_t,async_allocator> bucketIdArray (displSize, stream);

    CUDA_CHECK(cudaMemsetAsync(bucketSizes.data(), 0,
        bucketSizes.size_bytes(), stream));
    
    CUDA_CHECK(cudaStreamSynchronize(stream));

    constexpr auto kThreadsPerBlock = 32;
    const auto blocks = (n / kThreadsPerBlock) + (n % kThreadsPerBlock ? 1 : 0);
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
            fprintf(stderr, "\ttemporary storage buffer increased from %zu"
                " bytes to (requested) %zu bytes\n",
                tmpStorage.size_bytes(), tmpStorageSize);
            tmpStorage.resize(tmpStorageSize);
        }
    };
    
    Buffer<uint32_t,managed_allocator> histogram (kBucketMaxSize + 1);
    CUDA_CHECK(cudaMemsetAsync(histogram.data(), 0,
        histogram.size_bytes(), stream));

    CUDA_CHECK(cudaMemsetAsync(tmpStorage.data(), 0,
        (kBucketMaxSize + 1) * sizeof (uint32_t), stream));

    dependency_histogram<<<blocks,kThreadsPerBlock,0,stream>>>(
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

    fprintf(stderr, "\tgreatest bucket size = %d\n", bucketGreatestSize);

    CUDA_CHECK(cudaMemPrefetchAsync((const void*) histogram.data(),
        histogram.size_bytes(), device, stream));

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
        (kBucketMaxSize + 1) * sizeof (uint32_t), stream));

    sort_buckets_descending<<<blocks,kThreadsPerBlock,0,stream>>>(
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
            _h = HashFunction(d(_mt), d(_mt));
            cofactor = std::gcd(keys.size(), _h.a);

            fprintf(stderr, "\thash(k) = %u * k + %u mod N; "
                "gcd(hashtable, h.A) = %u\n",
                _h.a, _h.b,
                cofactor);
        }
        while (cofactor > 1 && j < 16); // make it safe against infinite loops

#if 0
        result[0] = 0;

        // Test hash function feasibility
        feasibility_test<<<blocks,kThreadsPerBlock,4,stream>>>(
            result.data(), _h, buckets.data(), bucketSizes.data(), displSize
        );

        CUDA_CHECK(cudaStreamSynchronize(stream));

        if (result[0] != 0)
        {
            fprintf(stderr, "\tSKIPPING... impossible hash function\n");
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
                insert_keys<<<blocks, dim3(8, kThreadsPerBlock), 0, stream>>>(
                    (int*) result.data(),
                    _h,
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

        fprintf(stderr, "\t\tFailure count: %d\n", (int) result[0]);

        // Success?
        if (result[0] == 0)
        {
            fprintf(stderr, "\t\tSuccess with hash function number %d\n", hashId);
            break;
        }

        // In case of failure
        // Increase table size
        _tableSize = hashTable.size() + sizeIncreaseStep;
        hashTable.resize(_tableSize);
        fprintf(stderr, "\tN <- %u\n", _tableSize);
    }

    std::unordered_set<uint32_t> hashSet {};
    bool duplicate = false;
    for (auto& key : keys)
    {
        auto v = hash(key);
        // fprintf(stderr, "\t%8X => %u\n", key, v);
        duplicate |= hashSet.contains(v);
        hashSet.emplace(v);
    }

    if (duplicate)
        fprintf(stderr, "\tDUPLICATE FOUND!\n");
    else
        fprintf(stderr, "\tNo duplicate. Perfect Hash!\n");

    return (int) duplicate;
}

} // namespace cg::rt
