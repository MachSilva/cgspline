#include "CPURayTracer.h"

#include <graphics/Color.h>

namespace cg::rt
{

template<typename T> using limits = std::numeric_limits<T>;

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

    const int totalWork = _work.size();
    int workDone = 0;

    while (!_work.empty())
    {
        auto [x, y] = _work.back();
        _work.pop_back();
        auto x1 = std::min<uint16_t>(w, x + _options.tileSize);
        auto y1 = std::min<uint16_t>(h, y + _options.tileSize);
        renderTile(x, x1, y, y1);
        progress(++workDone, totalWork);
    }

    fprintf(stderr, "Ray tracing completed in %g ms\n", clock.time());

    _frame = nullptr;
    _scene = nullptr;
}

void CPURayTracer::renderTile(
    uint16_t X0, uint16_t X1,
    uint16_t Y0, uint16_t Y1) const
{
    for (int j = Y0; j < Y1; j++)
    {
        for (int i = X0; i < X1; i++)
        {
            // auto d = (_cameraNormalToWorld * pixelRayDirection(i, j)).versor();
            auto d = pixelRayDirection(i, j);
            Ray pixelRay
            {
                .origin = _cameraPosition,
                .direction = d,
                .tMin = 0.001f,
                .tMax = limits<float>::infinity()
            };
            vec3f c = trace(pixelRay, vec3f(1), 6);
            c.x = std::clamp(c.x, 0.0f, 1.0f);
            c.y = std::clamp(c.x, 0.0f, 1.0f);
            c.z = std::clamp(c.x, 0.0f, 1.0f);
            _frame->at(i, j) = pack_sRGB(c.x, c.y, c.z);
        }
    }
}

vec3f CPURayTracer::trace(const Ray& ray, vec3f attenuation, int depth) const
{
    Intersection hit
    {
        .object = nullptr,
        .t = ray.tMax,
    };
    int nearestObject = intersect(hit, ray);

    if (nearestObject < 0)
        return miss();

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
            .origin = M_1.transform(ray.origin),
            .direction = M_1.transformVector(ray.direction),
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
    _scene->topLevelBVH.intersect(hit0, ray0, fn);
    return nearestObject;
}

bool CPURayTracer::intersects(const Ray& ray0) const
{
    auto fn = [this](const Ray& ray, uint32_t i)
    {
        auto p = _scene->objects.get<Key::ePrimitive>(i);
        auto& M_1 = _scene->objects.get<Key::eWorld2LocalMatrix>(i);
        Ray localRay
        {
            .origin = M_1.transform(ray.origin),
            .direction = M_1.transformVector(ray.direction),
        };
        auto d = localRay.direction.length();
        localRay.tMin = ray.tMin * d;
        localRay.tMax = ray.tMax * d;
        d = 1 / d;
        localRay.direction *= d;
        return p->intersects(localRay);
    };
    return _scene->topLevelBVH.intersects(ray0, fn);
}



vec3f CPURayTracer::miss() const
{
    return _options.backgroundColor;
}

// vec3f CPURayTracer::anyHit(const Intersection& hit, const Ray& ray, uint32_t object)
// vec3f CPURayTracer::anyHit()
// {

// }

__host__ __device__
constexpr float G1(float dotNX, float k)
{
    return dotNX / (dotNX * (1 - k) + k);
}

// Smith's geometry shadow-mask function
__host__ __device__
constexpr float G(float dotNL, float dotNV, float r)
{
    const float r1 = r + 1;
    const float k = r1 * r1 * 0.125; // .125 = 1/8
    return G1(dotNL, k) * G1(dotNV, k);
}

// Microfacet normal distribution function
__host__ __device__
inline float D(float dotNH, float r)
{
    dotNH = std::max(dotNH, 0.0f);
    float a2 = powf(std::max(r, 1e-3f), 4); // a = r^2; a^2 = r^4
    float d = dotNH * dotNH * (a2 - 1) + 1; // dot(H,N)^2 * (a^2 - 1) + 1
    return a2 / (std::numbers::pi_v<float> * d*d);
}

// Schlick's approximation for Fresnel reflectance for each wavelength
__host__ __device__
inline vec3f schlick(const vec3f& R0, float dotLH)
{
    float b = 1 - dotLH;
    float b2 = b*b;
    return R0 + (vec3f(1) - R0) * (b2*b2*b); // powf(1 - dotLH, 5)
}

// Original Fresnel reflectance for each wavelength
//   eta: relative refractive index
// TODO
// __host__ __device__
// inline vec3f fresnel(const vec3f& R0, float dotLH, float eta)
// {
//     return R0 + (vec3f(1) - R0) * powf(1 - dotLH, 5);
// }

template<typename T, std::floating_point real>
__host__ __device__
constexpr T mix(const T& a, const T& b, real t)
{
    return (real(1) - t) * a + t * b;
}

__host__ __device__
inline vec3f BRDF_diffuse(const Material& m)
{
    // Lambertian diffuse
    return m.diffuse * std::numbers::inv_pi_v<float>;
}

__host__ __device__
inline vec3f BRDF_specular(
    const vec3f& L,
    const vec3f& V,
    const vec3f& N,
    float dotNV,
    float dotNL,
    const Material& m)
{
    vec3f H = (L + V).versor();
    return (
        schlick(m.specular, vec3f::dot(L, H))
        * G(dotNL, dotNV, m.roughness)
        * D(vec3f::dot(H, N), m.roughness)
    ) * (1 / (4 * dotNL * dotNV));
}

__host__ __device__
vec3f BRDF(
    const vec3f& I,
    const vec3f& L,
    const vec3f& V,
    const vec3f& N,
    float dotNV,
    float dotNL,
    const Material& m)
{
    vec3f d = BRDF_diffuse(m);
    vec3f s = BRDF_specular(L, V, N, dotNV, dotNL, m);
    return I * mix(d, s, m.metalness) * dotNL;
}

/**
 * @brief Computes a vector from the point @a P to the light.
 * 
 * @param [out] d Light distance (infinity if the light is directional)
 * @param [out] L An unit vector from the point @a P to the light @a L
 * @param P The point
 * @param light The light
 * @return bool If point @a P is illuminated by the light
 */
inline
bool CPURayTracer::lightVector(float& d, vec3f& L, const vec3f& P, const Light& light) const
{
    if (light.isDirectional())
    {
        L = -light.direction;
        d = limits<float>::infinity();
        return true;
    }
    L = light.position - P;
    d = L.length();

    if (d < 1e-14f || (light.range > 0 && d > light.range))
        return false;

    L *= (1 / d);
    if (light.isPoint())
        return true;

    // spot
    float DL = vec3f::dot(light.direction, L);
    return DL < 0 && light.angle > 2 * acosf(DL);
}

vec3f CPURayTracer::lightColor(float d, const Light& light) const
{
    // directional light
    if (light.isDirectional())
        return light.color;

    float range = light.range;
    float f;
    if (range == 0) // infinite range
    {
        f = 1 / d;
        f *= f;
    }
    else
    {
        f = d / range;
        f = 1 + f * (f - 2); // (1 - f)^2
    }
    return light.color * f;
}

vec3f CPURayTracer::closestHit(const Intersection& hit, const Ray& ray,
    vec3f attenuation, uint32_t object, int depth) const
{
    const auto p = _scene->objects.get<Key::ePrimitive>(object);
    const auto& m = _scene->objects.get<Key::eMaterial>(object);
    const auto& M_1 = _scene->objects.get<Key::eWorld2LocalMatrix>(object);
    vec3f color {0};

    // m.specular.xyz = max(m.specular.xyz, vec3f(0.04));

    // From the point to the camera; BRDF uses this vector orientation.
    vec3f V = - ray.direction;
    // vec3f N = p->normal(hit);
    vec3f N = (mat3f(M_1).transposed() * p->normal(hit)).versor();

    bool backfaced = false;
    float dotNV = vec3f::dot(N, V);

    if (dotNV < 0)
    {
        backfaced = true;
        N = -N;
        dotNV = -dotNV;
    }

    vec3f P = ray.origin + hit.t * ray.direction;

    const auto& lights = _scene->lights;
    for (int i = 0; i < lights.size(); i++)
    {
        vec3f I;
        vec3f L;
        float d;

        if (!lightVector(d, L, P, lights[i]))
            continue;

        const float dotNL = vec3f::dot(N, L);
        if (dotNL < 1e-14f) // Avoid division by zero and backfaced lights
            continue;

        // Shadow
        Ray r1 { .origin = P + _options.eps * L, .direction = L, .tMax = d };
        if (intersects(r1))
            continue;

        I = lightColor(d, lights[i]);
        color += BRDF(I, L, V, N, dotNV, dotNL, m);
    }

    color *= std::numbers::pi_v<float> * attenuation;

    if (depth <= 0)
        return color;

    // Reflection
    constexpr float minRadiance = 0x1p-8f;
    vec3f reflectance = schlick(m.specular, dotNV);
    vec3f a = attenuation * reflectance;
    if (a.max() > minRadiance)
    {
        // I = -V
        vec3f R = (-V) + 2 * dotNV * N;
        Ray r
        {
            .origin = P + _options.eps * R,
            .direction = R,
            .tMax = limits<float>::infinity()
        };
        // color += m.opacity * trace(r, attenuation, depth - 1);
        color += trace(r, a, depth - 1);
    }

    // Refraction
    // vec3f transmittance = vec3f(1) - reflectance;
    // // Metals end up absorbing transmitted light
    // a = attenuation * transmittance * m.transparency * (1 - m.metalness);
    // if (a.max() > minRadiance)
    // {
    //     float eta = backfaced
    //         ? (1 / m.refractiveIndex)
    //         : m.refractiveIndex;
    //     // I = -V

    //     // color += 
    // }

    return color;
}

} // namespace cg::rt
