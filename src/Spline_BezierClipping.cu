#include "Spline.h"
#include <geometry/Triangle.h>

#include <cuda/std/array>
#include <cuda/std/cstddef>
#include <cuda/std/limits>
#include <cuda/std/utility>
#include "rt/Frame.h"
#include "rt/StaticConfig.h"
#include "SplineMat.h"

namespace cg::rt
{

extern __managed__ Frame* g_BezierClippingHeatMap;

} // namespace cg::rt

namespace cg::spline
{

using ::cuda::std::numeric_limits;

#ifdef SPL_BC_STATS
namespace stats
{
std::vector<BCData> g_BezierClippingData;
bool g_BezierClippingEnable = true;
} // namespace stats
#endif

/**
 * @brief Performs Bézier clipping on an already projected non-rational patch.
 *        Intersection points lie in the origin (0,0) of the projected space.
 * 
 * @param onHit Callback to receive all the intersection coordinates.
 * @param patch Bézier patch in 2D with control points (x, y).
 * @param tol Required precision for each dimension.
 * @retval Returns if an intersection was found.
 */
HOST DEVICE
bool doBezierClipping2D(std::predicate<vec2> auto onHit,
    const vec2 patch[16],
    float tol = 0x1p-12f);

HOST DEVICE
bool doBezierClipping(Intersection& hit,
    const Ray3f& ray,
    const vec4 buffer[],
    const uint32_t patch[16],
    float tol)
{
    // project patch into a 2D plane perpendicular to the ray direction
    vec2 patch2D[16];

    // find two planes whose intersection is a line that contains the ray
    const auto& d = ray.direction;

    const vec3 axis0 = d.cross(d + vec3(0,0,8)).versor();
    const vec3 axis1 = d.cross(axis0).versor();

    // quat q (vec3(0,0,1).cross(d), 1 + d.z);
    // q.normalize();
    // const vec3 axis0 = q.rotate(vec3(1,0,0)).versor();
    // const vec3 axis1 = q.rotate(vec3(0,1,0)).versor();

    for (int i = 0; i < 16; i++)
    {
// #if !defined(__CUDA_ARCH__)
        const vec4 P = buffer[patch[i]];
// #else
//         const vec4 P
//         {
//             __ldg((const float4*) buffer + __ldg(patch+i))
//         };
// #endif
        const vec3 Q {P.x, P.y, P.z};
        patch2D[i] =
        {
            distance(axis0, ray.origin, Q),
            distance(axis1, ray.origin, Q)
        };
    }

#if defined(SPL_BC_STATS) && !defined(__CUDA_ARCH__)
    stats::BCData* data = nullptr;
    if (stats::g_BezierClippingEnable)
    {
        stats::g_BezierClippingData.push_back({ .patch2D = std::to_array(patch2D) });
        data = &stats::g_BezierClippingData.back();
    }
#endif

    PatchRef S (buffer, patch);
    auto onHit = [&](vec2 e) -> bool
    {
        auto V = project(mat::interpolate(S, e.x, e.y)) - ray.origin;
        float t = V.length();

#if defined(SPL_BC_STATS) && !defined(__CUDA_ARCH__)
        if (data)
            data->hits.push_back({ .distance = t, .coord = e});
#endif

        if (t < hit.distance && V.dot(ray.direction) > 0)
        {
            hit.distance = t;
            hit.p = {e.x, e.y};
            return true;
        }
        return false;
    };
    return doBezierClipping2D(onHit, patch2D, tol);
}

/**
 * @brief Find where does the x-axis intersects with points @a A and @a B
 *        and store the result in @a result.
 * 
 * @note result.x <= result.y.
 * @param result The range of the intersection relative to the x-axis.
 * @param A Point A
 * @param B Point B
 * @return true  Intersection found
 * @return false No intersection
 */
_SPL_CONSTEXPR
bool xAxisIntersection(vec2& result, const vec2 A, const vec2 B)
{
    constexpr float eps = numeric_limits<float>::epsilon();
    // find where does the x-axis intersects with points A and B
    // (1-t)A + tB = 0
    // A - tA + tB = 0
    // A + (B - A)t = 0
    // Ay + (By - Ay)t = 0
    // Ay = (Ay - By)t, Ay != By
    // Ay / (Ay - By) = t, Ay != By
    float q = A.y - B.y;
    if (fabs(q) > eps)
    { // not zero
        float t = A.y / q;
        if (0.0f <= t && t <= 1.0f)
        {
            float s = (1.0f-t)*A.x + t*B.x;
            result = {s, s};
            return true;
        }
    }
    else if (fabs(A.y) <= eps)
    {
        result = {fmin(A.x, B.x), fmax(A.x, B.x)};
        // result = 0.5f*A.x + 0.5f*B.x;
        return true;
    }
    return false;
}

// Returns a positive value if Q is a vector obtained by rotating P counter
// clockwise by less than 180º; otherwise, returns a negative value; or zero
// if both vectors are parallel.
_SPL_CONSTEXPR
float isCCW(vec2 P, vec2 Q)
{
    return P.x*Q.y - P.y*Q.x;
}

// Returns a vector perpendicular to the segment AB. A and B are points.
_SPL_CONSTEXPR
vec2 perpendicular(vec2 A, vec2 B)
{
    return {A.y - B.y, B.x - A.x}; // rotate (B - A) by 90° degrees ccw
}

static HOST DEVICE
bool doBezierClipping2D_host(std::predicate<vec2> auto onHit,
    const vec2 patch[16],
    float tol)
{
    struct State
    {
        vec2 patch[16];
        vec2 min;
        vec2 max;
        vec2 size;
        enum Side
        {
            eU = 0,
            eV = 1
        } cutside;
    };

#if defined(SPL_BC_STATS) && !defined(__CUDA_ARCH__)
    int maxDepth = 1; // we already start with one element
#endif
    bool found = false;
#if !defined(__CUDA_ARCH__)
    std::vector<State> S;
#else
    namespace std = ::cuda::std;
    FixedArray<State,16> S; // a stack
#endif

    S.push_back({
        .min = {0,0}, .max = {1,1},
        .size = {1,1}, .cutside = State::eU
    });

    {
        auto& p = S.back().patch;
        for (int i = 0; i < 16; i++)
            p[i] = patch[i];
    }

    float distancePatch[16];
    Patch d (distancePatch);

#if defined(SPL_BC_STATS) && !defined(__CUDA_ARCH__)
    stats::BCData* searchData = nullptr;
    if (stats::g_BezierClippingEnable)
        searchData = &stats::g_BezierClippingData.back();
#endif

    do
    {
        auto &e = S.back();
        auto p = Patch(e.patch);

#if defined(SPL_BC_STATS) && !defined(__CUDA_ARCH__)
        if (searchData)
            searchData->steps.push_back({
                .L = {0, 0},
                .min = e.min,
                .max = e.max,
                .cutside = e.cutside == State::eU ? 'U' : 'V',
                .lower = NAN,
                .upper = NAN
            });
#endif
    
        // check tolerance
        float d0 = (p.point(3,0) - p.point(0,3)).length();
        float d1 = (p.point(3,3) - p.point(0,0)).length();
        if (d0 < tol && d1 < tol)
        {
            found |= approximateIntersection(onHit, p, e.min, e.max, e.size);
            S.pop_back();
            continue;
        }

        // find regions to clip
        constexpr float over3[] {0.0f, 1.0f/3.0f, 2.0f/3.0f, 1.0f};
        float lower = 1.0f, upper = 0.0f;
        // compute where does the convex hull intersect the x-axis
        float ytop[4]; // y top
        float ybot[4]; // y bottom

        {
            float distancePatch[16];
            Patch d (distancePatch);

            {
                // find line L
                // vector parallel to the line L
                vec2 L = p.point(3,3) - p.point(0,0);
                // optimized calculation
                vec2 D = p.point(0,3) - p.point(3,0);
                if (e.cutside == State::eU)
                    L += D;
                else // State::eV
                    L -= D;
                L.normalize();

                // vector perpendicular to the line L
                const vec2 N = {-L.y, L.x};

                // compute the distance patch
                for (int i = 0; i < 16; i++)
                    d[i] = N.dot(p[i]);
            }

            // one branch instead of 16
            if (e.cutside == State::eU)
            {
                for (int i = 0; i < 4; i++)
                {
                    ybot[i] = ytop[i] = d.point(i,0);
                    for (int k = 1; k < 4; k++)
                    {
                        float y = d.point(i,k);
                        ytop[i] = fmax(ytop[i], y);
                        ybot[i] = fmin(ybot[i], y);
                    }
                }
            }
            else // State::eV
            {
                for (int i = 0; i < 4; i++)
                {
                    ybot[i] = ytop[i] = d.point(0,i);
                    for (int k = 1; k < 4; k++)
                    {
                        float y = d.point(k,i);
                        ytop[i] = fmax(ytop[i], y);
                        ybot[i] = fmin(ybot[i], y);
                    }
                }
            }
        }
        
        // find convex hull and its intersection
        // graham scan
        {
            const vec2 hull[]
            {
                // bottom points
                {over3[2], ybot[2]},
                {over3[3], ybot[3]},
                // top points (right to left)
                {over3[3], ytop[3]},
                {over3[2], ytop[2]},
                {over3[1], ytop[1]},
                {over3[0], ytop[0]},
                // duplicate the first point to close the cycle
                {over3[0], ybot[0]}
            };
            // first and second points
            vec2 P1 {over3[0], ybot[0]};
            vec2 P2 {over3[1], ybot[1]};
            for (const vec2& P3 : hull)
            {
                if (isCCW(P2 - P1, P3 - P1) >= 0)
                {
                    if (vec2 s; xAxisIntersection(s, P1, P2))
                    {
                        // so far, performed better than plain ifs
                        lower = fmin(lower, s.x);
                        upper = fmax(upper, s.y);
                    }
                    P1 = P2;
                }
                P2 = P3;
            }
            if (vec2 s; xAxisIntersection(s, P1, P2))
            {
                lower = fmin(lower, s.x);
                upper = fmax(upper, s.y);
            }
        }

        // preventing the algorithm from creating long and narrow subpatches
        lower = lower * 0x0FFp-8f;
        // upper = upper * 0x101p-8f;
        upper = upper + (1 - upper) * 0x001p-8f;

#if defined(SPL_BC_STATS) && !defined(__CUDA_ARCH__)
        if (searchData)
        {
            auto& step = searchData->steps.back();
            step.L = L;
            step.lower = lower;
            step.upper = upper;
        }
#endif

        float delta = upper - lower;
        assert(delta <= 1.0f);
        assert(lower >= 0.0f && upper <= 1.0f);

        if (delta < 0)
        { // no intersection
            S.pop_back();
        }
        else if (delta > 0.8f)
        { // clipping too small; thus, subdivide
            State s1;
            for (int i = 0; i < 16; i++)
                s1.patch[i] = e.patch[i];
            if (e.cutside == State::eU)
            {
                float hs = 0.5f * e.size.x;
                float hx = e.min.x + hs;
                s1.max = e.max;
                s1.min = { hx, e.min.y };
                e.max = { hx, e.max.y };
                // e.min = e.min;
                s1.size = e.size = { hs, e.size.y };
                s1.cutside = e.cutside = State::eV;
                subpatchU(e.patch, 0.0f, 0.5f);
                subpatchU(s1.patch, 0.5f, 1.0f);
            }
            else // State::eV
            {
                float hs = 0.5f * e.size.y;
                float hy = e.min.y + hs;
                s1.max = e.max;
                s1.min = { e.min.x, hy };
                e.max = { e.max.x, hy };
                // e.min = e.min;
                s1.size = e.size = { e.size.x, hs };
                s1.cutside = e.cutside = State::eU;
                subpatchV(e.patch, 0.0f, 0.5f);
                subpatchV(s1.patch, 0.5f, 1.0f);
            }
            S.push_back(std::move(s1));

#if defined(SPL_BC_STATS) && !defined(__CUDA_ARCH__)
            maxDepth = std::max(maxDepth, (int) S.size());
#endif
        }
        else
        { // clip
            if (e.cutside == State::eU)
            {
                float u0 = e.min.x + e.size.x * lower;
                float u1 = e.max.x - e.size.x * (1.0f - upper);
                subpatchU(e.patch, lower, upper);
                e.min.x = u0;
                e.max.x = u1;
                e.size.x = u1 - u0;
                e.cutside = State::eV;
            }
            else // State::eV
            {
                float v0 = e.min.y + e.size.y * lower;
                float v1 = e.max.y - e.size.y * (1.0f - upper);
                subpatchV(e.patch, lower, upper);
                e.min.y = v0;
                e.max.y = v1;
                e.size.y = v1 - v0;
                e.cutside = State::eU;
            }
        }
    }
    while (!S.empty());

#if defined(SPL_BC_STATS) && !defined(__CUDA_ARCH__)
    if (searchData)
    {
        auto& m = searchData->maxStackDepth;
        m = std::max(m, maxDepth);
    }
#endif

    return found;
}

static_assert(numeric_limits<float>::is_iec559);

_SPL_CONSTEXPR
uint8_t get_f32_biased_exp(float f) noexcept
{
    auto k = reinterpret_cast<uint32_t&>(f);
    return uint8_t(k >> 23);
}

/**
 * Positive interval in an uint16_t
 * 
 * Bits [0-4]: interval length float (no sign)
 * - [0-4] exponent, bias = 31 (no nans, no infinities)
 *   - uint 0 => zero
 *   - uint 1 to 31 => 2^-30 to 2^0
 * Bits [5-15]: interval mid point, 11 bits
 * - normalized unsigned integer: 0x0 to 0x0800 => 0.0 to 1.0
 */
_SPL_CONSTEXPR
uint16_t range16_from_midpoint(float midpoint, float length) noexcept
{
    auto e = (uint16_t)get_f32_biased_exp(length);
    if (e != 0)
        e += 31 - 127;

    uint16_t m = round(midpoint * 0x0400);
    return (m << 5) | e & 0x1F;
}

_SPL_CONSTEXPR
void range16_extract(uint16_t value, float& midpoint, float& length) noexcept
{
    if (auto e = value & 0x1F; e == 0)
        length = 0;
    else
        length = exp2f(e - 31);

    midpoint = 0x1p-11f * (value >> 5);
}

/**
 * Positive interval in an uint16_t
 * 
 * Bits [0-5]: interval length float (no sign)
 * - [0-5] exponent, bias = 63 (no nans, no infinities)
 *   - uint 0 => zero
 *   - uint 1 to 63 => 2^-62 to 2^0
 * Bits [6-15]: interval mid point
 * - [6-10] exponent, bias = 31 (no nans, no infinities)
 *   - uint 0 => zero
 *   - uint 1 to 31 => 2^-30 to 2^0
 * - [11-15] mantissa (with implicit leading bit), 5 bits
 */
// constexpr int c_Range16_LengthExpBias = 63;
// constexpr int c_Range16_MidExpBias = 63;
// _SPL_CONSTEXPR
// uint16_t range16_from_midpoint(float midpoint, float length) noexcept
// {
//     auto e = (uint16_t)get_f32_biased_exp(length);
//     if (e != 0)
//     {
//         e += c_Range16_LengthExpBias - 127;

//         // if the mantissa is too high (two higher bits), then round up
//         // auto m = reinterpret_cast<uint32_t&>(length);
//         // if (m & 0x00400000)
//         //     ++e;
//     }
//     auto e2 = (uint16_t)get_f32_biased_exp(midpoint);
//     if (e2 != 0)
//         e2 += c_Range16_MidExpBias - 127;
//     auto m = reinterpret_cast<uint32_t&>(midpoint) & 0x007FFFFF;
//     uint16_t r = (m >> 7) & 0xF000;
//     return r | ((e2 & 0x3F) << 6) | (e & 0x3F);
// }

// _SPL_CONSTEXPR
// void range16_extract(uint16_t value, float& midpoint, float& length) noexcept
// {
//     if (auto e = value & 0x3F; e == 0)
//         length = 0;
//     else
//         length = exp2f(e - c_Range16_LengthExpBias);
//     if (auto e = (value >> 6) & 0x3F; e == 0)
//         midpoint = 0;
//     else
//     {
//         // auto m = (value >> 11) | 0x20;
//         auto m = (value >> 12) | 0x10;
//         midpoint = ldexpf(m, e - c_Range16_MidExpBias - 4);
//     }
// }

#define SPL_STACK_TYPE_NORMAL 0
#define SPL_STACK_TYPE_HALF 1
#define SPL_STACK_TYPE_RANGE16 2

#define SPL_STACK_TYPE SPL_STACK_TYPE_HALF

static DEVICE
bool doBezierClipping2D_device(std::predicate<vec2> auto onHit,
    const vec2 patch[16],
    float tol)
{
    namespace std = ::cuda::std;
    using namespace mat;
    struct State
    {
        vec2 min;
        vec2 max;
    };

    enum Side
    {
        eU = 0,
        eV = 1
    };

    bool found = false;
    int cutsideStack = 0;
#if SPL_STACK_TYPE == SPL_STACK_TYPE_NORMAL
    ArrayAdaptor<State> S ((State*)alloca(32 * sizeof (State)), 32); // a stack
    S.push_back({ .min = {0,0}, .max = {1,1} });
    
#elif SPL_STACK_TYPE == SPL_STACK_TYPE_HALF
    struct State4
    {
        half2 mid, len;
    };
    auto S = (State4*)alloca(32 * sizeof (State4)); // a stack
    int S_len = 0;

    State e { .min = {0,0}, .max = {1,1} };
#elif SPL_STACK_TYPE == SPL_STACK_TYPE_RANGE16
    auto S = (uint32_t*)alloca(32 * sizeof (uint32_t)); // a stack
    int S_len = 0;

    State e { .min = {0,0}, .max = {1,1} };
#endif

    vec2 buffer[16];
    Patch p (buffer);

    for (int i = 0; i < 16; i++)
        buffer[i] = patch[i];

    do
    {
#if SPL_STACK_TYPE == SPL_STACK_TYPE_NORMAL
        auto &e = S.back();
#endif
        const bool cutside = cutsideStack & 1;
    
        // check tolerance
        float d0 = (p.point(3,0) - p.point(0,3)).length();
        float d1 = (p.point(3,3) - p.point(0,0)).length();
        if (d0 < tol && d1 < tol)
        {
            found |= approximateIntersection(onHit, p, e.min, e.max, e.max - e.min);
        }
        else
        {
            // find regions to clip
            constexpr float over3[] {0.0f, 1.0f/3.0f, 2.0f/3.0f, 1.0f};
            float lower = 1.0f, upper = 0.0f;
            // compute where does the convex hull intersect the x-axis
            float ytop[4]; // y top
            float ybot[4]; // y bottom

            {
                float distancePatch[16];
                Patch d (distancePatch);

                {
                    // find line L
                    // vector parallel to the line L
                    vec2 L = p.point(3,3) - p.point(0,0);
                    // optimized calculation
                    vec2 D = p.point(0,3) - p.point(3,0);
                    if (cutside == Side::eU)
                        L += D;
                    else // Side::eV
                        L -= D;
                    L.normalize();

                    // vector perpendicular to the line L
                    const vec2 N = {-L.y, L.x};

                    // compute the distance patch
                    for (int i = 0; i < 16; i++)
                        d[i] = N.dot(p[i]);
                }

                for (int i = 0; i < 4; i++)
                {
                    float y[]
                    {
                        cutside == Side::eU ? d.point(i,0) : d.point(0,i),
                        cutside == Side::eU ? d.point(i,1) : d.point(1,i),
                        cutside == Side::eU ? d.point(i,2) : d.point(2,i),
                        cutside == Side::eU ? d.point(i,3) : d.point(3,i)
                    };
                    ybot[i] = fmin(fmin(y[0],y[1]),fmin(y[2],y[3]));
                    ytop[i] = fmax(fmax(y[0],y[1]),fmax(y[2],y[3]));
                }
            }

            // find convex hull and its intersection
            // graham scan
            {
                const vec2 hull[]
                {
                    // bottom points
                    {over3[2], ybot[2]},
                    {over3[3], ybot[3]},
                    // top points (right to left)
                    {over3[3], ytop[3]},
                    {over3[2], ytop[2]},
                    {over3[1], ytop[1]},
                    {over3[0], ytop[0]},
                    // duplicate the first point to close the cycle
                    {over3[0], ybot[0]}
                };
                // first and second points
                vec2 P1 {over3[0], ybot[0]};
                vec2 P2 {over3[1], ybot[1]};
                for (const vec2& P3 : hull)
                {
                    if (isCCW(P2 - P1, P3 - P1) >= 0)
                    {
                        if (vec2 s; xAxisIntersection(s, P1, P2))
                        {
                            // so far, performed better than plain ifs
                            lower = fmin(lower, s.x);
                            upper = fmax(upper, s.y);
                        }
                        P1 = P2;
                    }
                    P2 = P3;
                }
                if (vec2 s; xAxisIntersection(s, P1, P2))
                {
                    lower = fmin(lower, s.x);
                    upper = fmax(upper, s.y);
                }
            }

            __builtin_assume (lower >= 0.0f);
            __builtin_assume (upper <= 1.0f);
                    
            // preventing the algorithm from creating long and narrow subpatches
            lower = lower * 0x0FFp-8f;
            // upper = upper * 0x101p-8f;
            upper = upper + (1 - upper) * 0x001p-8f;

            float delta = upper - lower;
            if (delta > 0.8f)
            { // clipping too small; thus, subdivide
                State s1;
                if (cutside == Side::eU)
                {
                    // float hs = 0.5f * csize.x;
                    float hx = 0.5f * (e.min.x + e.max.x);
                    s1.max = e.max;
                    s1.min = { hx, e.min.y };
                    e.max.x = hx;
                    // e.min.x = e.min.x;
                    cutsideStack <<= 1;
                    cutsideStack |= 3;
                    // subpatchU(buffer, 0.0f, 0.5f);
                    subpatchU(buffer, 0.5f, 1.0f);
                }
                else // Side::eV
                {
                    // float hs = 0.5f * csize.y;
                    float hy = 0.5f * (e.min.y + e.max.y);
                    s1.max = e.max;
                    s1.min = { e.min.x, hy };
                    e.max.y = hy;
                    // e.min.y = e.min.y;
                    cutsideStack <<= 1;
                    cutsideStack &= ~3;
                    // subpatchV(buffer, 0.0f, 0.5f);
                    subpatchV(buffer, 0.5f, 1.0f);
                }
#if SPL_STACK_TYPE == SPL_STACK_TYPE_NORMAL
                S.push_back(s1);
#elif SPL_STACK_TYPE == SPL_STACK_TYPE_HALF
                if (S_len >= 32)
                    __trap();

                State4 s;
                s.mid.x = 0.5f * (e.max.x + e.min.x);
                s.mid.y = 0.5f * (e.max.y + e.min.y);
                s.len.x = e.max.x - e.min.x;
                s.len.y = e.max.y - e.min.y;
                S[S_len++] = s;

                e = s1;
#elif SPL_STACK_TYPE == SPL_STACK_TYPE_RANGE16
                if (S_len >= 32)
                    __trap();

                auto len = e.max - e.min;
                uint32_t r0 = range16_from_midpoint(e.min.x + 0.5f*len.x, len.x);
                uint32_t r1 = range16_from_midpoint(e.min.y + 0.5f*len.y, len.y);
                S[S_len++] = (r0 << 16) | (r1 & 0xFFFF);

                e = s1;
#endif
                continue;
            }
            else if (delta > 0)
            { // clip
                if (cutside == Side::eU)
                {
                    float sx = e.max.x - e.min.x;
                    float u0 = e.min.x + sx * lower;
                    float u1 = e.max.x - sx * (1.0f - upper);
                    subpatchU(buffer, lower, upper);
                    e.min.x = u0;
                    e.max.x = u1;
                    // e.cutside = Side::eV;
                }
                else // Side::eV
                {
                    float sy = e.max.y - e.min.y;
                    float v0 = e.min.y + sy * lower;
                    float v1 = e.max.y - sy * (1.0f - upper);
                    subpatchV(buffer, lower, upper);
                    e.min.y = v0;
                    e.max.y = v1;
                    // e.cutside = Side::eU;
                }
                cutsideStack ^= 1; // switch
                continue;
            }
        }

        // pop stack
        cutsideStack >>= 1;
#if SPL_STACK_TYPE == SPL_STACK_TYPE_NORMAL
        S.pop_back();
        if (!S.empty())
        {
            State s = S.back();
            patchCopy(buffer, patch, s.min, s.max);
        }
#elif SPL_STACK_TYPE == SPL_STACK_TYPE_HALF
        if (--S_len >= 0)
        {
            auto s = S[S_len];
            e.min.x = float(s.mid.x) - 0.5f * float(s.len.x);
            e.max.x = float(s.mid.x) + 0.5f * float(s.len.x);
            e.min.y = float(s.mid.y) - 0.5f * float(s.len.y);
            e.max.y = float(s.mid.y) + 0.5f * float(s.len.y);
            patchCopy(buffer, patch, e.min, e.max);
        }
#elif SPL_STACK_TYPE == SPL_STACK_TYPE_RANGE16
        if (--S_len >= 0)
        {
            auto s = S[S_len];
            float mid, len;
            range16_extract(s >> 16, mid, len);
            e.min.x = mid - 0.5f * len;
            e.max.x = mid + 0.5f * len;
            range16_extract((uint16_t)s, mid, len);
            e.min.y = mid - 0.5f * len;
            e.max.y = mid + 0.5f * len;
            patchCopy(buffer, patch, e.min, e.max);
        }
#endif
    }
#if SPL_STACK_TYPE == SPL_STACK_TYPE_NORMAL
    while (!S.empty());
#else
    while (S_len >= 0);
#endif

    return found;
}

HOST DEVICE
bool doBezierClipping2D(std::predicate<vec2> auto onHit,
    const vec2 patch[16],
    float tol)
{
#ifndef __CUDA_ARCH__
    return doBezierClipping2D_host(onHit, patch, tol);
#elif __CUDA_ARCH__ >= 500
    return doBezierClipping2D_device(onHit, patch, tol);
#else
#pragma error "Not yet implemented"
#endif
}

} // namespace cg::spline
