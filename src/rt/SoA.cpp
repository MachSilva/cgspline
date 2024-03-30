#include "SoA.h"


namespace cg::rt
{

#ifndef NDEBUG
// SoA specialization test
static void testfn()
{
    {
        const SoA<uint32_t, int, float, int, double> soa;
        auto& a = soa.data<0>();
        auto& b = soa.data<1>();
        auto& c = soa.data<2>();
        auto& d = soa.data<3>();
        static_assert(std::is_same_v<decltype(a), int*const&>);
        static_assert(std::is_same_v<decltype(b), float*const&>);
        static_assert(std::is_same_v<decltype(c), int*const&>);
        static_assert(std::is_same_v<decltype(d), double*const&>);
    }
    {
        SoA<uint32_t, int, float, int, double> soa;
        auto& a = soa.data<0>();
        auto& b = soa.data<1>();
        auto& c = soa.data<2>();
        auto& d = soa.data<3>();
        static_assert(std::is_same_v<decltype(a), int*&>);
        static_assert(std::is_same_v<decltype(b), float*&>);
        static_assert(std::is_same_v<decltype(c), int*&>);
        static_assert(std::is_same_v<decltype(d), double*&>);
    }
}
#endif

} // namespace cg::rt
