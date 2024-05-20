#include "PBR.h"

namespace cg::rt
{

__host__ __device__
inline vec3 BRDF_diffuse(const Material& m)
{
    // Lambertian diffuse
    return m.diffuse * std::numbers::inv_pi_v<float>;
}

__host__ __device__
inline vec3 BRDF_specular(
    const vec3& L,
    const vec3& V,
    const vec3& N,
    float dotNV,
    float dotNL,
    float roughness,
    const vec3& R0)
{
    vec3 H = (L + V).versor();
    return schlick(R0, vec3::dot(L, H))
        * BRDF_microfacet(dotNV, dotNL, vec3::dot(H, N), roughness);
}

__host__ __device__
vec3 BRDF(
    const vec3& I,
    const vec3& L,
    const vec3& V,
    const vec3& N,
    float dotNV,
    float dotNL,
    const Material& m)
{
    vec3 d = BRDF_diffuse(m);
    vec3 s = BRDF_specular(L, V, N, dotNV, dotNL, m.roughness, m.specular);
    return I * mix(d, s, m.metalness) * dotNL;
}

__host__ __device__
bool lightVector(float& d, vec3& L, const vec3& P, const Light& light)
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
    float DL = vec3::dot(light.direction, L);
    return DL < 0 && light.angle > 2 * acosf(DL);
}

__host__ __device__
vec3 lightColor(float d, const Light& light)
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
