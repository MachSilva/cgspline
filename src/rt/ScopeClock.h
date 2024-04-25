#pragma once

#include <chrono>
#include <iostream>

namespace cg::rt
{

#ifdef _MSC_VER
#define SPL_TIME_THIS()
#else
#define SPL_TIME_THIS() ::cg::rt::ScopeClock __prof {__PRETTY_FUNCTION__, __LINE__}
#endif

struct ScopeClock
{
    using clock = std::chrono::high_resolution_clock;
    clock::time_point start {clock::now()};
    const char * name {};
    const int line {};
    std::function<void(std::chrono::microseconds)> callback;

    ScopeClock() noexcept = default;
    ScopeClock(std::function<void(std::chrono::microseconds)>&& printFunction)
        : callback{std::move(printFunction)} {}
    ScopeClock(const char * name, int line) noexcept
        : name{name}, line{line} {}
    ~ScopeClock()
    {
        using namespace std::chrono;
        auto end = clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        if (callback)
            callback(duration);
        else
            std::cerr << name << " " << duration.count() / 1000.0f << " ms\n";
    }
};

} // namespace cg::rt
