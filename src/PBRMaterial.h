#pragma once

#include <core/NameableObject.h>
#include <graphics/Color.h>
#include "Texture.h"

namespace cg
{

class PBRMaterial : public NameableObject
{
public:
    Color baseColor {Color::gray};
    float metalness {0.9f};
    float roughness {0.1f};

    Ref<gl::Texture> texBaseColor {nullptr};
    Ref<gl::Texture> texMetalRough {nullptr};
};

} // namespace cg
