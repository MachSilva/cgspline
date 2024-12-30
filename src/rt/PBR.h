#pragma once

#include <cuda/std/cmath>
#include <graphics/Color.h>
#include <numbers>
#include "RTNamespace.h"
#include "Scene.h"

namespace cg::rt
{

/**
 * @brief Computes a vector from the point @a P to the light.
 * 
 * @param [out] d Light distance (infinity if the light is directional)
 * @param [out] L An unit vector from the point @a P to the light @a L
 * @param P The point
 * @param light The light
 * @return bool If point @a P is illuminated by the light
 */
__host__ __device__
bool lightVector(float& d, vec3& L, const vec3& P, const Light& light);

__host__ __device__
vec3 lightColor(float d, const Light& light);

template<typename T, std::floating_point real>
__host__ __device__
constexpr T mix(const T& a, const T& b, real t)
{
    return (real(1) - t) * a + t * b;
}

__host__ __device__
constexpr vec3 reflect(vec3 I, vec3 N, float dotIN)
{
    return I - 2 * dotIN * N;
}

// TODO
// __host__ __device__
// constexpr vec3 refract(vec3 I, vec3 N, float eta)
// {
//     return {};
// }

__host__ __device__
constexpr auto squared(const auto& value)
{
    return value*value;
}

// Schlick's approximation for Fresnel reflectance for each wavelength
template<typename vec = float>
__host__ __device__
constexpr vec schlick(vec f0, float dotLH)
{
    float b = 1 - dotLH;
    float b2 = b*b;
    return f0 + (vec(1) - f0) * (b2*b2*b); // powf(1 - dotLH, 5)
}

__host__ __device__
vec3 BRDF(
    const vec3& L, //< light vector
    const vec3& V, //< view vector
    const vec3& N, //< normal vector
    float dotNV,
    float dotNL,
    const Material& m);

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
constexpr float D(float dotNH, float r)
{
    r = fmax(r, 1e-3f);
    float a = r*r;
    float a2 = a*a; // a = r^2; a^2 = r^4
    float d = dotNH * dotNH * (a2 - 1) + 1; // dot(H,N)^2 * (a^2 - 1) + 1
    return a2 / (std::numbers::pi_v<float> * d*d);
}

__host__ __device__
constexpr float BRDF_microfacet(
    float dotNV,
    float dotNL,
    float dotNH,
    float roughness)
{
    return G(dotNL, dotNV, roughness) * D(dotNH, roughness)
        / (4 * dotNL * dotNV);
}

__host__ __device__
constexpr vec3 BRDF_diffuse(const Material& m)
{
    // Lambertian diffuse
    return m.diffuse * std::numbers::inv_pi_v<float>;
}

__host__ __device__
constexpr vec3 BRDF_specular(
    const vec3& L,
    const vec3& V,
    const vec3& N,
    float dotNV,
    float dotNL,
    float roughness,
    const vec3& f0)
{
    vec3 H = (L + V).versor();
    return schlick(f0, vec3::dot(L, H))
        * BRDF_microfacet(dotNV, dotNL, vec3::dot(H, N), roughness);
}

__host__ __device__
constexpr vec3 BRDF_specular(
    float dotHL,
    float dotHN,
    float dotNV,
    float dotNL,
    float roughness,
    const vec3& f0)
{
    return schlick(f0, dotHL) * BRDF_microfacet(dotNV, dotNL, dotHN, roughness);
}

// Van Der Corput sequence
__host__ __device__
constexpr float vdc(uint32_t i) noexcept
{
#ifdef __CUDA_ARCH__
    i = __brev(i);
#else
    i = (i << 16u) | (i >> 16u);
    i = ((i & 0x00FF00FFu) << 8u) | ((i & 0xFF00FF00u) >> 8u);
    i = ((i & 0x0F0F0F0Fu) << 4u) | ((i & 0xF0F0F0F0u) >> 4u);
    i = ((i & 0x33333333u) << 2u) | ((i & 0xCCCCCCCCu) >> 2u);
    i = ((i & 0x55555555u) << 1u) | ((i & 0xAAAAAAAAu) >> 1u);
#endif
    return 0x1p-32f * (float)i; // = 2^-32 * i = ldexpf((float)i, -32)
}

} // namespace cg::rt
