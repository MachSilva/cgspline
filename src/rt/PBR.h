#pragma once

#include <cuda/std/cmath>
#include <graphics/Color.h>
#include <math/Vector4.h>
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
bool lightVector(float& d, vec3f& L, const vec3f& P, const Light& light);

__host__ __device__
vec3f lightColor(float d, const Light& light);

__host__ __device__
vec3f BRDF(
    const vec3f& I,
    const vec3f& L,
    const vec3f& V,
    const vec3f& N,
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
static inline float D(float dotNH, float r)
{
    dotNH = fmax(dotNH, 0.0f);
    float a2 = powf(fmax(r, 1e-3f), 4); // a = r^2; a^2 = r^4
    float d = dotNH * dotNH * (a2 - 1) + 1; // dot(H,N)^2 * (a^2 - 1) + 1
    return a2 / (std::numbers::pi_v<float> * d*d);
}

// Schlick's approximation for Fresnel reflectance for each wavelength
__host__ __device__
static inline vec3f schlick(const vec3f& R0, float dotLH)
{
    float b = 1 - dotLH;
    float b2 = b*b;
    return R0 + (vec3f(1) - R0) * (b2*b2*b); // powf(1 - dotLH, 5)
}

// Original Fresnel reflectance for each wavelength
//   eta: relative refractive index
// TODO
// __host__ __device__
// static inline vec3f fresnel(const vec3f& R0, float dotLH, float eta)
// {
//     return R0 + (vec3f(1) - R0) * powf(1 - dotLH, 5);
// }

template<typename T, std::floating_point real>
__host__ __device__
constexpr T mix(const T& a, const T& b, real t)
{
    return (real(1) - t) * a + t * b;
}

} // namespace cg::rt
