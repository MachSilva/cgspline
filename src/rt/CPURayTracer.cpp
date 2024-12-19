#include "CPURayTracer.h"

#include <graphics/Color.h>
#include "PBR.h"
#include "../Log.h"

namespace cg::rt
{

// using std::numeric_limits;

void CPURayTracer::progress(int done, int total) const
{
    const int m = int(_options.tileSize) * _options.tileSize;
    float p = (100.0f * done) / total;
    int a = m * done;
    int b = m * total;
    fprintf(stderr, "\rRay traced %d of %d pixels (%.2f%%)", a, b, p);
    if (done == total)
    {
        fprintf(stderr, "\nDone\n");
    }
}

void CPURayTracer::render(Frame* frame, const Camera* camera, const Scene* scene)
{
    if (_status.running.test_and_set())
    {
        log::warn("this ray tracer object is already busy");
        return;
    }

    _frame = frame;
    _scene = scene;
    uint32_t w = frame->width();
    uint32_t h = frame->height();

    clock.reset();
    clock.start();

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
        _workers.emplace_back(&CPURayTracer::work, this);
    }

    for (auto& t : _workers)
        t.detach();

    // while (!_work.empty())
    // {
    //     auto [x, y] = _work.back();
    //     _work.pop_back();
    //     auto x1 = std::min<uint16_t>(w, x + _options.tileSize);
    //     auto y1 = std::min<uint16_t>(h, y + _options.tileSize);
    //     renderTile(x, x1, y, y1);
    //     progress(++workDone, totalWork);
    // }

    // fprintf(stderr, "Ray tracing completed in %g ms\n", clock.time());
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
            log::info("Ray tracing completed in {} ms", clock.time());
            // Clear
            _frame = nullptr;
            _scene = nullptr;
            _status.running.clear();
            _status.running.notify_all();
        }
    }
}

void CPURayTracer::renderTile(
    uint16_t X0, uint16_t X1,
    uint16_t Y0, uint16_t Y1) const
{
    for (int j = Y0; j < Y1; j++)
    {
        for (int i = X0; i < X1; i++)
        {
            auto d = pixelRayDirection(i, j);
            Ray pixelRay
            {
                .origin = _cameraPosition,
                .direction = d,
                .tMin = 0.001f,
                .tMax = numeric_limits<float>::infinity()
            };
            vec3 c = trace(pixelRay, vec3(1), _options.recursionDepth);
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

    // Reflection
    constexpr float minRadiance = 0x1p-8f;
    float k = squared(m.roughness + 1) * 0.125;
    vec3 reflectance = (1 - k) * schlick(m.specular, dotNV);
    vec3 a = attenuation * reflectance;
    if (a.max() > minRadiance)
    {
        // I = -V
        vec3 R = (-V) + 2 * dotNV * N;
        Ray r
        {
            .origin = P + _options.eps * R,
            .direction = R,
            .tMax = numeric_limits<float>::infinity()
        };
        // color += m.opacity * trace(r, attenuation, depth - 1);
        color += trace(r, a, depth - 1);
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
