#include "PBR.h"

namespace cg::rt
{

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

__host__ __device__
bool lightVector(float& d, vec3f& L, const vec3f& P, const Light& light)
{
    if (light.isDirectional())
    {
        L = -light.direction;
        d = numeric_limits<float>::infinity();
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

__host__ __device__
vec3f lightColor(float d, const Light& light)
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

} // namespace cg::rt
