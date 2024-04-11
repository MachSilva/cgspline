#pragma once

#include <memory_resource>
#include <cuda/std/limits>
#include <cuda/std/span>

namespace cg::rt
{

using ::cuda::std::numeric_limits;
using ::cuda::std::span;
using ::std::pmr::memory_resource;
using ::std::pmr::polymorphic_allocator;
using ::std::pmr::vector;

} // namespace cg::rt
