#include "Spline.h"
#include <geometry/Triangle.h>

namespace cg::spline
{

std::vector<DebugData> g_DebugData;

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

    g_DebugData.push_back({ .patch2D = patch2D });
    auto& data = g_DebugData.back();

    if (doBezierClipping2D(hits, patch2D.data()))
    {
        PatchRef S (buffer, patch);
        data.hits.reserve(hits.size());
        for (auto &e : hits)
        {
            auto V = project(interpolate(S, e.x, e.y)) - ray.origin;
            float t = V.length();
            data.hits.push_back({ .distance = t, .coord = e});
            if (t < hit.distance)
            {
                hit.distance = t;
                hit.p = e;
                hit.userData = nullptr;
                hit.triangleIndex = 0;
            }
        }

        return true;
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
    // (By - Ay)t = -Ay
    // t = - Ay / (By - Ay), Ay != By
    // t = Ay / (Ay - By), Ay != By
    float q = A.y - B.y;
    if (std::abs(q) > eps)
    { // not zero
        float t = A.y / q;
        if (0.0f <= t && t <= 1.0f)
        {
            float s = (1.0f-t)*A.x+ t*B.x;
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

bool doBezierClipping2D(std::vector<vec2f>& hits,
    const vec2f patch[16],
    float tol)
{
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

    std::deque<State> S; // a stack

    S.push_back({
        .min = {0,0}, .max = {1,1},
        .size = {1,1}, .cutside = State::eU
    });
    std::copy_n(patch, 16, S.back().patch);

    float distancePatch[16];
    Patch d (distancePatch);

    // debug code
    auto& searchData = g_DebugData.back();

    do
    {
        auto &e = S.back();
        auto p = Patch(e.patch);

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
        // one branch instead of one per iteration (4*3 = 12)
        if (e.cutside == State::eU)
        {
            vec2f A, B, s;
            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < 3; i++)
                {
                    A = {over3[  i], d.point(  i,j)};
                    B = {over3[i+1], d.point(i+1,j)};
                    if (xAxisIntersection(s, A, B))
                    {
                        lower = std::min(lower, s.x);
                        upper = std::max(upper, s.y);
                    }
                }
                // A.x <= B.x => s.x <= s.y
                A = {over3[0], d.point(0,j)};
                B = {over3[3], d.point(3,j)};
                if (xAxisIntersection(s, A, B))
                {
                    lower = std::min(lower, s.x);
                    upper = std::max(upper, s.y);
                }
            }
        }
        else // State::eV
        {
            vec2f A, B, s;
            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < 3; i++)
                {
                    A = {over3[  i], d.point(j,  i)};
                    B = {over3[i+1], d.point(j,i+1)};
                    if (xAxisIntersection(s, A, B))
                    {
                        lower = std::min(lower, s.x);
                        upper = std::max(upper, s.y);
                    }
                }
                A = {over3[0], d.point(j,0)};
                B = {over3[3], d.point(j,3)};
                if (xAxisIntersection(s, A, B))
                {
                    lower = std::min(lower, s.x);
                    upper = std::max(upper, s.y);
                }
            }
        }

        // debug code
        searchData.steps.push_back({
            .L = L,
            .min = e.min,
            .max = e.max,
            .cutside = e.cutside == State::eU ? 'U' : 'V',
            .lower = lower,
            .upper = upper
        });

        float delta = upper - lower;
        assert(delta <= 1.0f);
        assert(lower >= 0.0f && upper <= 1.0f);
        if (delta < 0)
        { // no intersection
            S.pop_back();
        }
        else if (delta > 0.8f)
        { // clipping too small; thus, subdivide
            // ...
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

            // check tolerance
            // float d0 = (p.point(3,0) - p.point(0,3)).length();
            // float d1 = (p.point(3,3) - p.point(0,0)).length();
            float d0 = e.size.x, d1 = e.size.y;
            if (d0 <= tol && d1 <= tol)
            { // report hit
                hits.push_back(0.5f * (e.min + e.max));
                S.pop_back();
            }
        }

        // debug code
        auto& m = searchData.maxStackDepth;
        m = std::max(m, (int) S.size());
    }
    while (!S.empty() && S.size() < 128);

    return !hits.empty();
}

} // namespace cg::spline
