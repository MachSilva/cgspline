#include "Spline.h"
#include <geometry/Triangle.h>

namespace cg::spline
{

#ifdef SPL_BC_STATS
namespace stats
{
std::vector<BCData> g_BezierClippingData;
bool g_BezierClippingEnable = true;
} // namespace stats
#endif

mat3f rotation(const vec3f& v, float angle)
{
    const auto c = std::cos(angle);
    const auto s = std::sin(angle);
    const auto c1 = 1.0f - c;
    const auto xx = v.x * v.x;
    const auto yy = v.y * v.y;
    const auto zz = v.z * v.z;
    const auto xy = v.x * v.y;
    const auto xz = v.x * v.z;
    const auto yz = v.y * v.z;
    const auto sx = s * v.x;
    const auto sy = s * v.y;
    const auto sz = s * v.z;
    return mat3f
    { // columns
        {c1 * xx + c,  c1 * xy + sz, c1 * xz - sy},
        {c1 * xy - sz, c1 * yy + c,  c1 * yz + sx},
        {c1 * xz + sy, c1 * yz - sx, c1 * zz + c}
    };
}

/**
 * @brief Computes the distance between a point P and a plane represented by the
 *        normal vector N and origin O.
 * 
 * @param N The plane normal vector (must be unit length)
 * @param O The plane origin point
 * @param P The point whose distance is being measured
 * @return float The computed distance
 */
static inline
float distance(const vec3f& N, const vec3f& O, const vec3f& P)
{
    return N.dot(P - O);
}

/**
 * @brief Computes the distance between a point P and a line represented by the
 *        normal vector N and origin O.
 * 
 * @param N The line normal vector (must be unit length)
 * @param O The line origin point
 * @param P The point whose distance is being measured
 * @return float The computed distance
 */
static inline
float distance(const vec2f& N, const vec2f& O, const vec2f& P)
{
    return N.dot(P - O);
}

bool doBezierClipping(Intersection& hit,
    const Ray3f& ray,
    const vec4f buffer[],
    const uint32_t patch[16],
    float tol)
{
    thread_local std::vector<vec2f> hits;
    hits.clear();

    // project patch into a 2D plane perpendicular to the ray direction
    // vec2f patch2D[16];
    std::array<vec2f,16> patch2D;

    // find two planes whose intersection is a line that contains the ray
    const auto& d = ray.direction;
    const vec3f axis0 = d.cross(d + vec3f{0, 0, 10.f}).versor();
    const vec3f axis1 = d.cross(axis0).versor();

    for (int i = 0; i < 16; i++)
    {
        const auto& P = buffer[patch[i]];
        const vec3f Q {P.x, P.y, P.z};
        patch2D[i] =
        {
            distance(axis0, ray.origin, Q),
            distance(axis1, ray.origin, Q)
        };
    }

#ifdef SPL_BC_STATS
    stats::BCData* data = nullptr;
    if (stats::g_BezierClippingEnable)
    {
        stats::g_BezierClippingData.push_back({ .patch2D = patch2D });
        data = &stats::g_BezierClippingData.back();
    }
#endif

    if (doBezierClipping2D(hits, patch2D.data(), tol))
    {
        PatchRef S (buffer, patch);
        bool found = false;
        for (auto &e : hits)
        {
            auto V = project(interpolate(S, e.x, e.y)) - ray.origin;
            float t = V.length();
            if (t < hit.distance && V.dot(ray.direction) > 0)
            {
                found = true;
                hit.distance = t;
                hit.p = e;
            }

#ifdef SPL_BC_STATS
            if (data)
                data->hits.push_back({ .distance = t, .coord = e});
#endif
        }

        return found;
    }

    return false;
}

/**
 * @brief Find where does the x-axis intersects with points @a A and @a B
 *        and store the result in @a s.
 * 
 * @note If A.x <= B.x then s.x <= s.y.
 * @param result The range of the intersection relative to the x-axis.
 * @param A Point A
 * @param B Point B
 * @return true  Intersection found
 * @return false No intersection
 */
static
bool xAxisIntersection(vec2f& result, const vec2f A, const vec2f B)
{
    constexpr float eps = std::numeric_limits<float>::epsilon();
    // find where does the x-axis intersects with points A and B
    // (1-t)A + tB = 0
    // A - tA + tB = 0
    // A + (B - A)t = 0
    // Ay + (By - Ay)t = 0
    // Ay = (Ay - By)t, Ay != By
    // Ay / (Ay - By) = t, Ay != By
    float q = A.y - B.y;
    if (std::abs(q) > eps)
    { // not zero
        float t = A.y / q;
        if (0.0f <= t && t <= 1.0f)
        {
            float s = (1.0f-t)*A.x + t*B.x;
            result = {s, s};
            return true;
        }
    }
    else if (std::abs(A.y) <= eps)
    {
        result = {A.x, B.x};
        return true;
    }
    return false;
}

// Returns a positive value if Q is a vector obtained by rotating P counter
// clockwise by less than 180º; otherwise, returns a negative value; or zero
// if both vectors are parallel.
static inline
float isCCW(vec2f P, vec2f Q)
{
    return P.x*Q.y - P.y*Q.x;
}

// Returns a vector perpendicular to the segment AB. A and B are points.
static inline // constexpr
vec2f perpendicular(vec2f A, vec2f B)
{
    return {A.y - B.y, B.x - A.x}; // rotate (B - A) by 90° degrees ccw
}

static inline
bool triangleContainsOrigin(vec2f A, vec2f B, vec2f C)
{
    vec2f p = perpendicular(A, B);
    vec2f q = perpendicular(B, C);
    vec2f r = perpendicular(C, A);

    if (std::signbit(isCCW(p, -r))) // isCCW(AB, AC)
        return p.dot(A) >= 0 && q.dot(B) >= 0 && r.dot(C) >= 0;

    return p.dot(-A) >= 0 && q.dot(-B) >= 0 && r.dot(-C) >= 0;
}

static inline
bool triangleOriginIntersection(vec2f &coord, vec2f P, vec2f A, vec2f B)
{
    constexpr float eps = std::numeric_limits<float>::epsilon();

    vec2f p = A - P;
    vec2f q = B - P;
    float det = p.x*q.y - q.x*p.y;

    if (math::isZero(det))
        return false;

    det = 1.0f / det;

    // 2x2 inversion matrix
    vec2f m0 = vec2f( q.y, -q.x); // first line
    vec2f m1 = vec2f(-p.y,  p.x); // second line

    auto u = det * m0.dot(-P);
    if (u < 0 || u > 1)
        return false;

    auto v = det * m1.dot(-P);
    if (v < 0 || u + v > 1)
        return false;

    coord.set(u, v);
    return true;
}

bool doBezierClipping2D(std::vector<vec2f>& hits,
    const vec2f patch[16],
    float tol)
{
    constexpr float eps = std::numeric_limits<float>::epsilon();
    struct State
    {
        vec2f patch[16];
        vec2f min;
        vec2f max;
        vec2f size;
        enum Side
        {
            eU = 0,
            eV = 1
        } cutside;
    };

    bool stackFreeze = false;
    std::deque<State> S; // a stack

    S.push_back({
        .min = {0,0}, .max = {1,1},
        .size = {1,1}, .cutside = State::eU
    });
    std::copy_n(patch, 16, S.back().patch);

    float distancePatch[16];
    Patch d (distancePatch);

#ifdef SPL_BC_STATS
    stats::BCData* searchData = nullptr;
    if (stats::g_BezierClippingEnable)
        searchData = &stats::g_BezierClippingData.back();
#endif

    do
    {
        auto &e = S.back();
        auto p = Patch(e.patch);

#ifdef SPL_BC_STATS
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
        // float d0 = (p.point(3,0) - p.point(0,3)).length();
        // float d1 = (p.point(3,3) - p.point(0,0)).length();
        if (e.size.x < tol && e.size.y < tol)
        {
            // there are cases where is impossible to clip the patch because
            // it is already so small that it is a plane and the intersection
            // has been already found
            // thus, the infinite subdivision must be stopped

            // intersect triangles to avoid multiple intersection problem
            auto p00 = p.point(0,0);   // (u,v) = (0,0)
            auto p10 = p.point(3,0);   // (1,0)
            auto p01 = p.point(0,3);   // (0,1)
            auto p11 = p.point(3,3);   // (1,1)

            vec2f coord;
            // if (triangleOriginIntersection(coord, p10, p01, p00)
            //     || triangleOriginIntersection(coord, p01, p10, p11))
            // {
            //     hits.push_back(0.5f * (e.min + e.max));
            // }

            if (triangleOriginIntersection(coord, p00, p10, p01))
            {
                hits.push_back(e.min + coord * e.size);
            }
            if (triangleOriginIntersection(coord, p11, p01, p10))
            {
                hits.push_back(e.max - coord * e.size);
            }

            // if (triangleContainsOrigin(p10, p01, p00)
            //     || triangleContainsOrigin(p01, p10, p11))
            // {
            //     hits.push_back(0.5f * (e.min + e.max));
            // }
            S.pop_back();
            continue;
        }

        vec2f L0, L1;
        // find line L
        if (e.cutside == State::eU)
        {
            L0 = p.point(0,3) - p.point(0,0);
            L1 = p.point(3,3) - p.point(3,0);
        }
        else // State::eV
        {
            L0 = p.point(3,0) - p.point(0,0);
            L1 = p.point(3,3) - p.point(0,3);
        }
        // vector parallel to the line L
        const vec2f L = (L0 + L1).versor();
        // vector perpendicular to the line L
        const vec2f N = {-L.y, L.x};

        // compute the distance patch
        for (int i = 0; i < 16; i++)
            d[i] = N.dot(p[i]);

        // find regions to clip
        constexpr float over3[] {0.0f, 1.0f/3.0f, 2.0f/3.0f, 1.0f};
        float lower = 1.0f, upper = 0.0f;
        // compute where does the convex hull intersect the x-axis
        float ytop[4]; // y top
        float ybot[4]; // y bottom
        // one branch instead of 16
        if (e.cutside == State::eU)
        {
            for (int i = 0; i < 4; i++)
            {
                ybot[i] = ytop[i] = d.point(i,0);
                for (int k = 1; k < 4; k++)
                {
                    float y = d.point(i,k);
                    ytop[i] = std::max(ytop[i], y);
                    ybot[i] = std::min(ybot[i], y);
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
                    ytop[i] = std::max(ytop[i], y);
                    ybot[i] = std::min(ybot[i], y);
                }
            }
        }
        // find convex hull and its intersection
        // graham scan
        vec2f hull[9];
        int len = 0;
        // first and second points
        hull[len++] = {over3[0], ybot[0]};
        hull[len++] = {over3[1], ybot[1]};
        // bottom points
        for (int i = 2; i < 4; i++)
        {
            vec2f P3 {over3[i], ybot[i]};
            auto& P2 = hull[len-1];
            auto& P1 = hull[len-2];
            if (isCCW(P2 - P1, P3 - P1) >= 0)
                hull[len++] = P3;
            else
                hull[len-1] = P3;
        }
        // first top point (right to left)
        hull[len++] = {over3[3], ytop[3]};
        // top points
        for (int i = 2; i >= 0; i--)
        {
            vec2f P3 = {over3[i], ytop[i]};
            auto& P2 = hull[len-1];
            auto& P1 = hull[len-2];
            if (isCCW(P2 - P1, P3 - P1) >= 0)
                hull[len++] = P3;
            else
                hull[len-1] = P3;
        }
        // duplicate the first point to close the cycle
        hull[len++] = hull[0];
        // find intersection and the clip range
        for (int i = 1; i < len; i++)
        {
            vec2f A = hull[i-1];
            vec2f B = hull[i];
            vec2f s;
            if (xAxisIntersection(s, A, B))
            {
                if (s.x > s.y)
                    std::swap(s.x, s.y);
                lower = std::min(lower, s.x);
                upper = std::max(upper, s.y);
            }
        }

        // TODO Improve this step
        if (lower <= upper)
        { // preventing the algorithm from creating long and narrow subpatches
            constexpr float minspan = 0.1f;
            auto mid = 0.5f * (lower + upper);
            if (upper - lower < minspan)
            {
                lower = mid - 0.5f * minspan;
                upper = mid + 0.5f * minspan;
            }
        }

#ifdef SPL_BC_STATS
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
            std::copy_n(e.patch, 16, s1.patch);
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

            if (S.size() > 127)
            {
                fprintf(stderr,
                    "warning: the stack has too many elements (%llu)\n", S.size());
                break;
            }
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

#ifdef SPL_BC_STATS
        if (searchData)
        {
            auto& m = searchData->maxStackDepth;
            m = std::max(m, (int) S.size());
        }
#endif
    }
    while (!S.empty());

    return !hits.empty();
}

} // namespace cg::spline
