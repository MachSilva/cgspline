#pragma once

#include <core/NameableObject.h>
#include <graphics/Color.h>
#include "Texture.h"

namespace cg
{

class PBRMaterial : public NameableObject
{
public:
    Color diffuse {Color::gray};
    Color specular {Color::darkGray}; // Reflectance
    float metalness {0.9f};
    float roughness {0.1f};

    Ref<gl::Texture> diffuseTexture {nullptr};
    Ref<gl::Texture> specularTexture {nullptr};
    Ref<gl::Texture> textureMetalRough {nullptr};
};

} // namespace cg
