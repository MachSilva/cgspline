#include "CPURayTracer.h"

#include <graphics/Color.h>
#include "PBR.h"
#include "../Log.h"

namespace cg::rt
{

struct
{
    std::uniform_real_distribution<float> unit {0, 1}; // unit distribution
    std::ranlux24_base rng {(uint32_t)rand()};
} thread_local g_PerThread;

static vec3 randomSphereVec3()
{
    auto& t = g_PerThread;
    constexpr float eps = 1e-7f;
    vec3 p;
    do
    {
        p = {eps + t.unit(t.rng), eps + t.unit(t.rng), eps + t.unit(t.rng)};
    }
    while (p.squaredNorm() > 1.0f);
    return p;
}

/**
 * @brief Random microfacet normal in polar coordinates.
 * 
 * @param alpha
 * @return vec2 polar coordinates in \f$ [0,\pi/2] \cross [0,2\pi] \f$
 */
static vec2 randomMicrofacet(float alpha)
{
    auto& t = g_PerThread;
    float x = t.unit(t.rng);
    float y = t.unit(t.rng);
    // Microfacet normal distribution
    return
    {
        atan2f(sqrtf(1 - x), alpha * sqrtf(x)),
        2 * std::numbers::pi_v<float> * y
    };
}

void CPURayTracer::progress(int done, int total) const
{
    const int m = int(_options.tileSize) * _options.tileSize;
    float p = (100.0f * done) / total;
    int a = m * done;
    int b = m * total;
    fprintf(stderr, "\rRay traced %d of %d pixels (%.2f%%)\t", a, b, p);
    if (done == total)
    {
        fprintf(stderr, "\nDone\n");
    }
}

void CPURayTracer::render(Frame* frame, const Camera* camera, const Scene* scene)
{
    // if (_status.running.test_and_set())
    if (this->running())
    {
        log::warn("this ray tracer object is already busy");
        return;
    }

    _frame = frame;
    _scene = scene;
    uint32_t w = frame->width();
    uint32_t h = frame->height();

    _status.started = clock::now();

    // ceil(w / _options.tileSize)
    _countX = (w / _options.tileSize) + (w % _options.tileSize ? 1 : 0);
    _countY = (h / _options.tileSize) + (h % _options.tileSize ? 1 : 0);

    // Distribute the work
    _work.clear();
    for (int j = _countY - 1; j >= 0; j--)
        for (int i = _countX - 1; i >= 0; i--)
            _work.emplace_back(i * _options.tileSize, j * _options.tileSize);

    // Camera data
    _cameraPosition = camera->transform.position;
    _cameraToWorld = camera->transform.matrix();
    _cameraNormalToWorld = camera->transform.normalMatrix();

    auto fovy = camera->fieldOfView;
    auto halfHeight = tanf(0.5f * fovy);
    _half_dy = halfHeight / h;
    _half_dx = _half_dy; // * camera->aspectRatio;
    _topLeftCorner.x = -_half_dx * (w - 1);
    _topLeftCorner.y = halfHeight - _half_dy;

    if (_options.flipYAxis)
    {
        _topLeftCorner.y = -_topLeftCorner.y;
        _half_dy = -_half_dy;
    }

    _status.totalWork = _work.size();
    _status.workDone = 0;

    _workers.clear();
    _workers.reserve(_options.threads);
    for (int i = 0; i < _options.threads; i++)
    {
        // _workers.emplace_back(&CPURayTracer::work, this);
        _workers.emplace_back(
            std::async(std::launch::async, &CPURayTracer::work, this)
        );
    }

    // for (auto& t : _workers)
    // {
    //     t.detach();
    // }
}

void CPURayTracer::work()
{
    uint16_t x, y;
    while (true)
    {
        {
            std::lock_guard lck (_workMutex);
            if (_work.empty())
                return;
            auto& t = _work.back();
            x = t.x, y = t.y;
            _work.pop_back();
        }
        auto x1 = std::min<uint16_t>(_frame->width(), x + _options.tileSize);
        auto y1 = std::min<uint16_t>(_frame->height(), y + _options.tileSize);
        renderTile(x, x1, y, y1);
        _status.workDone.fetch_add(1);

        // Last one?
        if (_status.workDone.load() == _status.totalWork)
        {
            _status.finished = clock::now();
            std::chrono::duration<double,std::milli>
                duration = _status.finished - _status.started;
            log::info("Ray tracing completed in {} ms", duration.count());
            // Clear
            _frame = nullptr;
            _scene = nullptr;
            // _status.running.clear();
            // _status.running.notify_all();
        }
    }
}

void CPURayTracer::stop()
{
    {
        std::lock_guard lck (_workMutex);
        _work.clear();
    }
    // _status.running.wait(true);
}

void CPURayTracer::renderTile(
    uint16_t X0, uint16_t X1,
    uint16_t Y0, uint16_t Y1) const
{
    const auto n = _options.nSamples;
    const auto n_inv = 1.0f / n;
    for (int j = Y0; j < Y1; j++)
    {
        for (int i = X0; i < X1; i++)
        {
            vec3 c {0};
            for (int s = 0; s < n; s++)
            {
                float s0 = s * n_inv;
                float s1 = vdc(i);
                auto d = pixelRayDirection(i + 2*_half_dx*s0, j + 2*_half_dy*s1);
                Ray pixelRay
                {
                    .origin = _cameraPosition,
                    .direction = d,
                    .tMin = 0.001f,
                    .tMax = numeric_limits<float>::infinity()
                };
                c += trace(pixelRay, vec3(1), _options.recursionDepth);
            }
            c *= n_inv;
            c.x = std::clamp(c.x, 0.0f, 1.0f);
            c.y = std::clamp(c.y, 0.0f, 1.0f);
            c.z = std::clamp(c.z, 0.0f, 1.0f);
            _frame->at(i, j) = pack_sRGB(c.x, c.y, c.z);
        }
    }
}

vec3 CPURayTracer::trace(const Ray& ray, vec3 attenuation, int depth) const
{
    Intersection hit
    {
        .object = nullptr,
        .t = ray.tMax,
    };
    int nearestObject = intersect(hit, ray);
    _status.rays.fetch_add(1, std::memory_order_relaxed);

    if (nearestObject < 0)
        return attenuation * miss();

    _status.hits.fetch_add(1, std::memory_order_relaxed);

    // Is opaque?
    // auto& m = _scene->objects.get<Key::eMaterial>(nearestObject);
    // if (m.opacity < 0.999f)
    // {
    //     // anyHit();
    // }

    return closestHit(hit, ray, attenuation, nearestObject, depth);
}

int CPURayTracer::intersect(Intersection& hit0, const Ray& ray0) const
{
    int nearestObject = -1;
    auto fn = [&nearestObject, this]
    (Intersection& hit, const Ray& ray, uint32_t i)
    {
        Intersection localHit;
        auto p = _scene->objects.get<Key::ePrimitive>(i);
        auto& M_1 = _scene->objects.get<Key::eWorld2LocalMatrix>(i);
        Ray localRay
        {
            .origin = project(M_1.transform(vec4(ray.origin, 1))),
            .direction = mat3(M_1).transform(ray.direction),
        };
        auto d = localRay.direction.length();
        localRay.tMin = ray.tMin * d;
        localRay.tMax = ray.tMax * d;
        d = 1 / d;
        localRay.direction *= d;
        localHit.t = localRay.tMax;
        if (p->intersect(localHit, localRay))
        {
            auto t = localHit.t * d;
            if (t < hit.t)
            {
                hit.coordinates = localHit.coordinates;
                hit.index = localHit.index;
                hit.object = &p;
                hit.t = t;
                nearestObject = i;
                return true;
            }
        }
        return false;
    };
    _scene->topLevelBVH.hashIntersect(hit0, ray0, fn);
    return nearestObject;
}

bool CPURayTracer::intersect(const Ray& ray0) const
{
    auto fn = [this](const Ray& ray, uint32_t i)
    {
        auto p = _scene->objects.get<Key::ePrimitive>(i);
        auto& M_1 = _scene->objects.get<Key::eWorld2LocalMatrix>(i);
        Ray localRay
        {
            .origin = project(M_1.transform(vec4(ray.origin, 1))),
            .direction = mat3(M_1).transform(ray.direction),
        };
        auto d = localRay.direction.length();
        localRay.tMin = ray.tMin * d;
        localRay.tMax = ray.tMax * d;
        d = 1 / d;
        localRay.direction *= d;
        return p->intersect(localRay);
    };
    return _scene->topLevelBVH.hashIntersect(ray0, fn);
}

vec3 CPURayTracer::miss() const
{
    return _scene->backgroundColor;
}

// vec3 CPURayTracer::anyHit()
// {

// }

vec3 CPURayTracer::closestHit(const Intersection& hit, const Ray& ray,
    vec3 attenuation, uint32_t object, int depth) const
{
    const auto p = _scene->objects.get<Key::ePrimitive>(object);
    const auto& m = _scene->objects.get<Key::eMaterial>(object);
    const auto& M_1 = _scene->objects.get<Key::eWorld2LocalMatrix>(object);
    vec3 color {0};

    // From the point to the camera; BRDF uses this vector orientation.
    vec3 V = - ray.direction;
    // vec3 N = p->normal(hit);
    vec3 N = (mat3(M_1).transposed() * p->normal(hit)).versor();

    bool backfaced = false;
    float dotNV = vec3::dot(N, V);

    if (dotNV < 0)
    {
        backfaced = true;
        N = -N;
        dotNV = -dotNV;
    }

    vec3 P = ray.origin + hit.t * ray.direction;

    const auto& lights = _scene->lights;
    for (int i = 0; i < lights.size(); i++)
    {
        vec3 I;
        vec3 L;
        float d;

        if (!lightVector(d, L, P, lights[i]))
            continue;

        const float dotNL = vec3::dot(N, L);
        if (dotNL < 1e-14f) // Avoid division by zero and backfaced lights
            continue;

        // Shadow
        Ray r1 { .origin = P + _options.eps * L, .direction = L, .tMax = d };
        if (intersect(r1))
            continue;

        vec3 H = (L + V).versor();
        float dotHN = H.dot(N);
        float dotHL = H.dot(L);

        I = lightColor(d, lights[i]);
        color += I * mix(
            BRDF_diffuse(m),
            BRDF_specular(dotHL, dotHN, dotNV, dotNL, m.roughness, m.specular),
            m.metalness
        );
    }

    color *= std::numbers::pi_v<float> * attenuation;

    if (depth <= 0)
        return color;

    constexpr bool GGX = false;

    // Select random microfacet normal
    vec3 M;
    if constexpr (GGX)
    {
        vec3 t_s = N.cross(V).versor(); // tangent perpendicular
        vec3 t_p = N.cross(t_s).versor(); // tangent parallel
        vec2 c = randomMicrofacet(squared(m.roughness));
        M = sinf(c.x)*cosf(c.y)*t_s + sinf(c.x)*sinf(c.y)*t_p + cosf(c.x)*N;
        M.normalize();
    }
    else
    {
        M = (N + m.roughness * randomSphereVec3()).versor();
    }
    float dotMV = M.dot(V);

    // Reflection
    constexpr float minRadiance = 0x1p-8f;
    vec3 reflectance = schlick(m.specular, dotMV);
    {
        vec3 R = reflect(-V, M, -dotMV);
        float dotMN = M.dot(N);
        float dotNR = N.dot(R);

        if (dotNR < 1e-14f)
            return color;

        float g = G(dotNR, dotNV, m.roughness) * dotMV / (dotNV * dotMN);
        // if (g > 1.0f)
        // {
        //     log::error("g > 1; g = {}; M·V = {}, M·N = {}",
        //         g, dotMV, dotNV, dotMN);
        //     log::error("G(N·R = {}, N·V = {}, roughness = {}) = {}",
        //         dotNR, dotNV, m.roughness, G(dotNR, dotNV, m.roughness));
        //     std::abort();
        // }
        vec3 brdf = mix(
            BRDF_diffuse(m),
            vec3(reflectance * g),
            m.metalness
        );
        vec3 a = attenuation * brdf;
        if (a.max() > minRadiance)
        {
            Ray r
            {
                .origin = P + _options.eps * M,
                .direction = R,
                .tMax = numeric_limits<float>::infinity()
            };
            color += trace(r, a, depth - 1);
            // vec3 R = reflect(-V, M, -dotMV);
            // Ray r
            // {
            //     .origin = P + _options.eps * M,
            //     .direction = R,
            //     .tMax = numeric_limits<float>::infinity()
            // };
            // // float dotMN = M.dot(N);
            // // float g = G(N.dot(R), dotNV, m.roughness) * dotMV
            // //     / (dotNV * dotMN);
            // vec3 I = trace(r, attenuation, depth - 1);
            // color += I * mix(
            //     BRDF_diffuse(m),
            //     vec3(reflectance * dotMV),
            //     m.metalness
            // );
        }
    }

    // Refraction
    // Metals end up absorbing transmitted light
    // a = attenuation * transmittance * m.transparency * (1 - m.metalness);
    // if (a.max() > minRadiance)
    // {
    //     float eta = backfaced
    //         ? (1 / m.refractiveIndex)
    //         : m.refractiveIndex;
    //     // I = -V

    //     color += trace(r, a, depth - 1);
    // }

    return color;
}

} // namespace cg::rt
