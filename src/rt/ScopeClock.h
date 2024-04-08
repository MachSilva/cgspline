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
    const char * name;
    const int line;

    ScopeClock(const char * name, int line) noexcept
        : name{name}, line{line} {}
    ~ScopeClock()
    {
        using namespace std::chrono;
        auto end = clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        std::cerr << name << " " << duration.count() / 1000.0f << " ms\n";
    }
};

} // namespace cg::rt
