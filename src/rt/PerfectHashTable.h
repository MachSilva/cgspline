#pragma once

#include <array>
#include <cassert>
#include <random>
#include "CUDAUtility.h"
#include "RTNamespace.h"

namespace cg::rt
{

// Random prime numbers between 2^30 and 2^31
static constexpr std::array<int32_t,16> RANDOM_PRIMES31
{
    1933273841, 1439646163, 1293153791, 1150219159,
    1547529853, 1733961953, 1408002091, 1741593757,
    1525947499, 2036288231, 1372157351, 1312347037,
    1946609681, 1433388449, 1442234909, 2018826701,
};

// Random prime numbers between 2^31 and 2^32
static constexpr std::array<uint32_t,16> RANDOM_PRIMES32
{
    4174915957U, 2376772961U, 3641499011U, 2226070283U,
    2963417291U, 4089834317U, 3197253559U, 3312796573U,
    3054610121U, 3994827449U, 3128597239U, 4018580789U,
    2442028279U, 3978086807U, 3211402351U, 2816854373U,
};

struct HashFunction
{
    uint32_t a = 31;
    uint32_t b = 19;
    uint32_t p = RANDOM_PRIMES32[0];

public:
    __host__ __device__
    constexpr HashFunction() noexcept = default;

    __host__
    constexpr explicit
    HashFunction(uint32_t A, uint32_t B, int prime_seed = 0) noexcept
    {
        p = RANDOM_PRIMES32[prime_seed % RANDOM_PRIMES32.size()];
        a = A % p;
        b = B % p;
        // `a` cannot be zero
        if (a == 0)
            a = 31;
    }

    __host__ __device__
    constexpr uint32_t operator() (uint32_t value) const
    {
        return (a*value + b) % p;
    }
};

class PerfectHashTable
{
    HashFunction _h;
    std::mt19937 _mt;

    Buffer<uint32_t,managed_allocator> _displacement;
    uint32_t _displMask;
    uint32_t _tableSize;

public:
    static constexpr auto EMPTY = uint32_t(-1);

    PerfectHashTable(uint32_t seed = 42)
        : _mt{seed} {}

    /**
     * @param keys 
     * @param stream 
     * @return int Non-zero in case of failure
     */
    int build(span<const uint32_t> keys, cudaStream_t stream = 0);

    __host__ __device__
    uint32_t hash(uint32_t key) const
    {
        const uint32_t* ptr = _displacement.data();
        return (_h(key) + ptr[key & _displMask]) % _tableSize;
    }
};


} // namespace cg::rt
