//[]---------------------------------------------------------------[]
//|                                                                 |
//| Copyright (C) 2007, 2023 Paulo Pagliosa.                        |
//|                                                                 |
//| This software is provided 'as-is', without any express or       |
//| implied warranty. In no event will the authors be held liable   |
//| for any damages arising from the use of this software.          |
//|                                                                 |
//| Permission is granted to anyone to use this software for any    |
//| purpose, including commercial applications, and to alter it and |
//| redistribute it freely, subject to the following restrictions:  |
//|                                                                 |
//| 1. The origin of this software must not be misrepresented; you  |
//| must not claim that you wrote the original software. If you use |
//| this software in a product, an acknowledgment in the product    |
//| documentation would be appreciated but is not required.         |
//|                                                                 |
//| 2. Altered source versions must be plainly marked as such, and  |
//| must not be misrepresented as being the original software.      |
//|                                                                 |
//| 3. This notice may not be removed or altered from any source    |
//| distribution.                                                   |
//|                                                                 |
//[]---------------------------------------------------------------[]
//
// OVERVIEW: SceneReaderExt.h (formerly SceneReader.h)
// ========
// Class definition for scene reader.
//
// Author: Paulo Pagliosa (and contributors)
// Last revision: 11/07/2023
// Altered version last revision: 19/10/2023

#pragma once

#include <variant>
#include <reader/SceneReader.h>
#include "Surface.h"
#include "PBRMaterial.h"
#include "Ref.h"
#include "Texture.h"

namespace cg::util
{

using PBRMaterialMap = std::map<std::string,Ref<PBRMaterial>>;
using TriangleMeshMap = std::map<std::string,Ref<TriangleMesh>>;
using SurfaceMap = std::map<std::string,Ref<Surface>>;
using TextureMap = std::map<std::string,Ref<gl::Texture>>;

class SceneReaderExt : public parser::Reader
{
public:
    class Parser;

    MaterialMap materials;
    PBRMaterialMap pbrMaterials;
    TriangleMeshMap meshes;
    SurfaceMap surfaces;
    TextureMap textures;

    graph::Scene* scene() const
    {
        return _scene;
    }

    void execute() override;

protected:
    virtual graph::Scene* makeScene(const char *name) const;

private:
    Ref<graph::Scene> _scene;

    parser::Reader::Parser* makeParser() override;

    friend Parser;
}; // SceneReaderExt

class SceneReaderExt::Parser :
    public parser::Reader::Parser, public graph::SceneObjectBuilder
{
public:
    Parser(SceneReaderExt& reader) : Reader::Parser{reader}, _reader{&reader}
    {
        // do nothing
    }

    auto& reader() const
    {
        return *_reader;
    }

protected:
    using Base = parser::Reader::Parser;

    // Component type
    enum
    {
        _CAMERA,
        _LIGHT,
        _MESH,
        _SURFACE,
        lastComponentType
    };

    // Tokens
    enum
    {
        _AMBIENT = Base::lastToken,
        _ANGLE,
        _ASPECT,
        _BACKGROUND,
        _COLOR,
        _COMPONENT,
        _DEPTH,
        _DIFFUSE,
        _DIRECTIONAL,
        _ENVIRONMENT,
        _FALLOFF,
        _HEIGHT,
        _IOR,
        _MATERIAL,
        _OBJECT,
        _PARALLEL,
        _PERSPECTIVE,
        _POINT,
        _POSITION,
        _RANGE,
        _ROTATION,
        _SCALE,
        _SCENE,
        _SHINE,
        _SPECULAR,
        _SPOT,
        _TRANSFORM,
        _TRANSPARENCY,
        // extended token list
        _BEZIER,
        _PATH,
        _METALNESS,
        _ROUGHNESS,
        _PBRMATERIAL,
        _TEXTURE,
        lastToken
    };

    // Error codes
    enum
    {
        MATERIAL_ALREADY_DEFINED = Base::lastErrorCode,
        MULTIPLE_SCENE_DEFINITION,
        COMPONENT_ALREADY_DEFINED,
        INVALID_VALUE_FOR,
        EMPTY_MESH_NAME,
        COULD_NOT_FIND_MATERIAL,
        COULD_NOT_PARSE_COMPONENT,

        COULD_NOT_FIND_MESH,
        COULD_NOT_FIND_PBRMATERIAL,
        COULD_NOT_FIND_SURFACE,
        COULD_NOT_FIND_TEXTURE,
        MESH_ALREADY_DEFINED,
        PBRMATERIAL_ALREADY_DEFINED,
        SURFACE_ALREADY_DEFINED,
        TEXTURE_ALREADY_DEFINED,
        FILE_DOESNT_EXIST,
        ASSET_PARSING_FAILED,
        lastErrorCode
    };

    virtual Ref<graph::Component> parseComponent(int, graph::SceneObject&);

    std::variant<Color,Ref<gl::Texture>> matchColorOrTexture();

private:
    SceneReaderExt* _reader;

    void start() override;

    void preamble();
    void declaration();

    void parseScene();
    void parseSceneEnvironment();
    void parseObject(graph::SceneObject&);
    void parseObjectBlock(graph::SceneObject&);
    void parseChildObjectBlock(graph::SceneObject&);
    void parseTransform(graph::SceneObject&);
    void parseComponent(graph::SceneObject&);

    Ref<Material> parseMaterialDefinition();
    Ref<PBRMaterial> parsePBRMaterialDefinition();
    Ref<TriangleMesh> parseMeshDefinition();
    Ref<Surface> parseSurfaceDefinition();
    Ref<gl::Texture> parseTextureDefinition();

    MaterialMap::iterator matchMaterial();
    PBRMaterialMap::iterator matchPBRMaterial();
    TriangleMeshMap::iterator matchMesh();
    SurfaceMap::iterator matchSurface();
    TextureMap::iterator matchTexture();

    Ref<graph::CameraProxy> matchCamera();
    Ref<graph::LightProxy> matchLight();
    Ref<graph::PrimitiveProxy> matchPrimitive(int);

    DECLARE_KEYWORD_TABLE(SceneReaderExt::Parser);
    DECLARE_ERROR_MESSAGE_TABLE(SceneReaderExt::Parser);

}; // SceneReaderExt::Parser

} // namespace cg::util

