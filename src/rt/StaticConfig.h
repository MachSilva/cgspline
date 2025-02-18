#pragma once

#include <cuda_runtime.h>

// #define USE_MONTECARLO_SAMPLING
// #define RT_ENABLE_COUNTERS
// #define SPL_BC_HEATMAP

namespace cg::rt
{

#ifdef RT_ENABLE_COUNTERS
extern __shared__ int s_Counters[128];
#endif

} // namespace cg::rt
