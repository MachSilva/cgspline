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
// OVERVIEW: SceneReaderExt.cpp (formerly SceneReader.cpp)
// ========
// Source file for scene reader.
//
// Author: Paulo Pagliosa (and contributors)
// Last revision: 11/07/2023
// Altered version last revision: 19/10/2023

#include "SceneReaderExt.h"

#include <map>
#include <graph/SceneObjectBuilder.h>
#include <graphics/Assets.h>
#include <graphics/Application.h>
#include "GLBezierSurface.h"
#include "Surface.h"

namespace cg::util
{

void
SceneReaderExt::execute()
{
    Assets::initialize();

    materials.clear();
    meshes.clear();
    surfaces.clear();
    textures.clear();

    materials = Assets::materials();
    meshes = Assets::meshes();

    Reader::execute();
}

graph::Scene*
SceneReaderExt::makeScene(const char *name) const
{
    return graph::Scene::New(name);
}

parser::Reader::Parser*
SceneReaderExt::makeParser()
{
    return new SceneReaderExt::Parser(*this);
}

DEFINE_KEYWORD_TABLE(SceneReaderExt::Parser, parser::Reader::Parser)
    KEYWORD("ambient", _AMBIENT, 0)
    KEYWORD("angle", _ANGLE, 0)
    KEYWORD("aspect", _ASPECT, 0)
    KEYWORD("background", _BACKGROUND, 0)
    KEYWORD("camera", _COMPONENT, _CAMERA)
    KEYWORD("color", _COLOR, 0)
    KEYWORD("depth", _DEPTH, 0)
    KEYWORD("diffuse", _DIFFUSE, 0)
    KEYWORD("directional", _DIRECTIONAL, 0)
    KEYWORD("environment", _ENVIRONMENT, 0)
    KEYWORD("falloff", _FALLOFF, 0)
    KEYWORD("height", _HEIGHT, 0)
    KEYWORD("ior", _IOR, 0)
    KEYWORD("light", _COMPONENT, _LIGHT)
    KEYWORD("material", _MATERIAL, 0)
    KEYWORD("mesh", _COMPONENT, _MESH)
    KEYWORD("object", _OBJECT, 0)
    KEYWORD("parallel", _PARALLEL, 0)
    KEYWORD("perspective", _PERSPECTIVE, 0)
    KEYWORD("point", _POINT, 0)
    KEYWORD("position", _POSITION, 0)
    KEYWORD("range", _RANGE, 0)
    KEYWORD("rotation", _ROTATION, 0)
    KEYWORD("scale", _SCALE, 0)
    KEYWORD("scene", _SCENE, 0)
    KEYWORD("shine", _SHINE, 0)
    KEYWORD("specular", _SPECULAR, 0)
    KEYWORD("spot", _SPOT, 0)
    KEYWORD("transform", _TRANSFORM, 0)
    KEYWORD("transparency", _TRANSPARENCY, 0)
    // extended keyword list
    KEYWORD("bezier", _BEZIER, 0)
    KEYWORD("path", _PATH, 0)
    KEYWORD("metalness", _METALNESS, 0)
    KEYWORD("roughness", _ROUGHNESS, 0)
    KEYWORD("pbrmaterial", _PBRMATERIAL, 0)
    KEYWORD("surface", _COMPONENT, _SURFACE)
    KEYWORD("texture", _TEXTURE, 0)
END_KEYWORD_TABLE;

DEFINE_ERROR_MESSAGE_TABLE(SceneReaderExt::Parser, parser::Reader::Parser)
    ERROR_MESSAGE(MATERIAL_ALREADY_DEFINED,
        "Material '%s' already defined")
    ERROR_MESSAGE(COMPONENT_ALREADY_DEFINED,
        "Component '%s' already defined")
    ERROR_MESSAGE(INVALID_VALUE_FOR,
        "Invalid value for '%s'")
    ERROR_MESSAGE(EMPTY_MESH_NAME,
        "Empty mesh name")
    ERROR_MESSAGE(COULD_NOT_FIND_MATERIAL,
        "Could not find material '%s'")
    ERROR_MESSAGE(COULD_NOT_PARSE_COMPONENT,
        "Could not parse component '%s'")
    // extended message list
    ERROR_MESSAGE(COULD_NOT_FIND_MESH,
        "Could not find mesh '%s'")
    ERROR_MESSAGE(COULD_NOT_FIND_PBRMATERIAL,
        "Could not find PBR material '%s'")
    ERROR_MESSAGE(COULD_NOT_FIND_SURFACE,
        "Could not find surface '%s'")
    ERROR_MESSAGE(COULD_NOT_FIND_TEXTURE,
        "Could not find texture '%s'")
    ERROR_MESSAGE(MESH_ALREADY_DEFINED,
        "Mesh '%s' already defined")
    ERROR_MESSAGE(PBRMATERIAL_ALREADY_DEFINED,
        "PBR material '%s' already defined")
    ERROR_MESSAGE(SURFACE_ALREADY_DEFINED,
        "Surface '%s' already defined")
    ERROR_MESSAGE(TEXTURE_ALREADY_DEFINED,
        "Texture '%s' already defined")
    ERROR_MESSAGE(FILE_DOESNT_EXIST,
        "File '%s' does not exist")
    ERROR_MESSAGE(ASSET_PARSING_FAILED,
        "Asset (%s) parsing failed [%s]")
END_ERROR_MESSAGE_TABLE;

inline void
SceneReaderExt::Parser::preamble()
{
    for (;;)
        switch (_token)
        {
        case _INCLUDE:
            include();
            break;
        case _DEFINE:
            declaration();
            break;
        // Asset
        // {
        case _MATERIAL:
            (void)parseMaterialDefinition();
            break;
        case _PBRMATERIAL:
            (void)parsePBRMaterialDefinition();
            break;
        case _COMPONENT:
        {
            if (_tokenValue.integer == _MESH)
                (void)parseMeshDefinition();
            else if (_tokenValue.integer == _SURFACE)
                (void)parseSurfaceDefinition();
        } break;
        case _TEXTURE:
            (void)parseTextureDefinition();
            break;
        // }
        default:
            return;
        }
}

void
SceneReaderExt::Parser::start()
try
{
    preamble();
    if (_token == _SCENE)
    {
        if (_scene != nullptr)
            error(MULTIPLE_SCENE_DEFINITION);
        else
            parseScene();
    }
    if (_token != _EOF)
    {
        if (_token < 256)
            error(UNEXPECTED_CHAR, _token);
        else
            error(SYNTAX);
    }
}
catch (const std::exception &)
{
    _reader->_scene = nullptr;
    throw;
}

void
SceneReaderExt::Parser::declaration()
{
    // _DEFINE
    advance();

    auto name = matchName();

    if (_token == '=')
        advance();
    define(name, expression());
}

inline Ref<Material>
SceneReaderExt::Parser::parseMaterialDefinition()
{
    // _MATERIAL
    advance();

    auto name = matchString();
    if (_reader->materials.contains(name))
        error(MATERIAL_ALREADY_DEFINED, name.c_str());

    match('{');

    Reference material = new Material(Color::black);
    bool item = true;
    while (item)
        switch (_token)
        {
        case _AMBIENT:
            advance();
            material->ambient = matchColor();
            break;
        case _DIFFUSE:
            advance();
            material->diffuse = matchColor();
            break;
        case _SPOT:
            advance();
            material->spot = matchColor();
            break;
        case _SHINE:
            advance();
            if (auto shine = matchFloat(); shine <= 0)
                error(INVALID_VALUE_FOR, "shine");
            else
                material->shine = shine;
            break;
        case _SPECULAR:
            advance();
            material->specular = matchColor();
            break;
        case _TRANSPARENCY:
            advance();
            material->transparency = matchColor();
            break;
        case _IOR:
            advance();
            if (auto ior = matchFloat(); ior <= 0)
                error(INVALID_VALUE_FOR, "ior");
            else
                material->ior = ior;
            break;
        default:
            item = false;
        }
    match('}');
    return _reader->materials[name] = material;
}

inline Ref<PBRMaterial>
SceneReaderExt::Parser::parsePBRMaterialDefinition()
{
    // _PBRMATERIAL
    advance();

    auto name = matchString();
    if (_reader->pbrMaterials.contains(name))
        error(PBRMATERIAL_ALREADY_DEFINED, name.c_str());

    match('{');

    auto material = Reference(new PBRMaterial);
    float value;
    bool item = true;
    while (item)
        switch (_token)
        {
        case _COLOR:
            advance();
            material->baseColor = matchColor();
            break;
        case _METALNESS:
            advance();
            value = matchFloat();
            if (value < 0.0 || value > 1.0)
                error(INVALID_VALUE_FOR, "metalness");
            material->metalness = value;
            break;
        case _ROUGHNESS:
            advance();
            value = matchFloat();
            if (value < 0.0 || value > 1.0)
                error(INVALID_VALUE_FOR, "roughness");
            material->roughness= value;
            break;
        case _TEXTURE:
            advance();
            material->texBaseColor = matchTexture()->second;
            break;
        case _NAME:
            if (matchName() == "metal_rough_texture")
            {
                material->texMetalRough = matchTexture()->second;
                break;
            }
        default:
            item = false;
        }
    match('}');
    return _reader->pbrMaterials[name] = material;
}

inline Ref<TriangleMesh>
SceneReaderExt::Parser::parseMeshDefinition()
{
    throw std::runtime_error("not implemented yet");
    return {};
}

inline Ref<Surface>
SceneReaderExt::Parser::parseSurfaceDefinition()
{
    // _SURFACE
    advance();

    auto name = matchString();
    if (_reader->surfaces.contains(name))
        error(SURFACE_ALREADY_DEFINED, name.c_str());

    match('{');
    match(_BEZIER);
    auto filename = matchString();
    auto file = _reader->_currentPath / filename;

    if (std::filesystem::exists(file) == false)
        error(FILE_DOESNT_EXIST, file.c_str());
    
    match('}');

    return _reader->surfaces[name] = GLBezierSurface::load(file.string().c_str());
}

inline Ref<gl::Texture>
SceneReaderExt::Parser::parseTextureDefinition()
{
    // _TEXTURE
    advance();

    auto name = matchString();
    if (_reader->textures.contains(name))
        error(TEXTURE_ALREADY_DEFINED, name.c_str());

    match('{');

    auto filename = matchString();
    auto file = _reader->_currentPath / filename;

    if (std::filesystem::exists(file) == false)
        error(FILE_DOESNT_EXIST, file.c_str());

    match('}');

    return _reader->textures[name] = gl::Texture::from(file.string().c_str());
}

MaterialMap::iterator
SceneReaderExt::Parser::matchMaterial()
{
    auto name = matchString();
    auto it = _reader->materials.find(name);

    if (it == _reader->materials.end())
    {
        error(COULD_NOT_FIND_MATERIAL, name.c_str());
    }
    return it;
}

PBRMaterialMap::iterator
SceneReaderExt::Parser::matchPBRMaterial()
{
    auto name = matchString();
    auto it = _reader->pbrMaterials.find(name);

    if (it == _reader->pbrMaterials.end())
    {
        error(COULD_NOT_FIND_PBRMATERIAL, name.c_str());
    }
    return it;
}

TriangleMeshMap::iterator
SceneReaderExt::Parser::matchMesh()
{
    auto name = matchString();
    auto it = _reader->meshes.find(name);

    if (it == _reader->meshes.end())
    {
        error(COULD_NOT_FIND_MESH, name.c_str());
    }
    return it;
}

SurfaceMap::iterator
SceneReaderExt::Parser::matchSurface()
{
    auto name = matchString();
    auto it = _reader->surfaces.find(name);

    if (it == _reader->surfaces.end())
    {
        error(COULD_NOT_FIND_SURFACE, name.c_str());
    }
    return it;
}

TextureMap::iterator
SceneReaderExt::Parser::matchTexture()
{
    auto name = matchString();
    auto it = _reader->textures.find(name);

    if (it == _reader->textures.end())
    {
        error(COULD_NOT_FIND_TEXTURE, name.c_str());
    }
    return it;
}

inline void
SceneReaderExt::Parser::parseSceneEnvironment()
{
    // _ENVIRONMENT
    advance();
    match('{');
    for (;;)
        if (_token == _AMBIENT)
        {
            advance();
            _scene->ambientLight = matchColor();
        }
        else if (_token == _BACKGROUND)
        {
            advance();
            _scene->backgroundColor = matchColor();
        }
        else if (_token == _TEXTURE)
        {
            advance();
            _reader->environment = matchTexture()->second;
        }
        else
            break;
    matchEndOfBlock();
}

void
SceneReaderExt::Parser::parseScene()
{
    // _SCENE
    advance();

    auto sceneName = matchOptionalString();
    auto scene = _reader->makeScene(sceneName.c_str());

    setScene(*scene);
    match('{');
    beginBlock();
    while (_token == _DEFINE)
        declaration();
    if (_token == _ENVIRONMENT)
        parseSceneEnvironment();
    parseObjectBlock(*scene->root());
    _reader->_scene = scene;
}

void
SceneReaderExt::Parser::parseObjectBlock(graph::SceneObject& object)
{
    for (;;)
        if (_token == _OBJECT)
            parseObject(object);
        else if (_token == _DEFINE)
            declaration();
        else
            break;
    matchEndOfBlock();
    endBlock();
}

inline void
SceneReaderExt::Parser::parseTransform(graph::SceneObject& object)
{
    // _TRANSFORM
    advance();
    match('{');
    for (auto t = object.transform();;)
        switch (_token)
        {
        case _POSITION:
            advance();
            t->setLocalPosition(matchVec3());
            break;
        case _ROTATION:
            advance();
            t->setLocalEulerAngles(matchVec3());
            break;
        case _SCALE:
            advance();
            if (auto s = matchVec3(); s.x > 0 && s.y > 0 && s.z > 0)
                t->setLocalScale(s);
            else
                error(INVALID_VALUE_FOR, "scale");
            break;
        default:
            matchEndOfBlock();
            return;
        }
}

inline void
SceneReaderExt::Parser::parseChildObjectBlock(graph::SceneObject& object)
{
    while (_token == _DEFINE)
        declaration();
    if (_token == _TRANSFORM)
        parseTransform(object);
    while (_token == _COMPONENT)
        parseComponent(object);
    parseObjectBlock(object);
}

void
SceneReaderExt::Parser::parseObject(graph::SceneObject& object)
{
    // _OBJECT
    advance();

    auto childName = matchOptionalString();
    auto child = createEmptyObject();

    if (!childName.empty())
        child->setName(childName.c_str());
    child->setParent(&object);
    if (_token == _COMPONENT)
        parseComponent(*child);
    else
    {
        match('{');
        beginBlock();
        parseChildObjectBlock(*child);
    }
}

inline Ref<graph::CameraProxy>
SceneReaderExt::Parser::matchCamera()
{
    // _COMPONENT (_CAMERA)
    advance();
    match('{');

    Ref<graph::CameraProxy> proxy{graph::CameraProxy::New()};

    for (auto camera = proxy->camera();;)
    {
        if (_token == _PERSPECTIVE)
            advance();
        else if (_token == _PARALLEL)
        {
            advance();
            camera->setProjectionType(Camera::Parallel);
        }
        switch (_token)
        {
        case _ANGLE:
            advance();
            if (auto angle = matchFloat(); angle > 0)
                camera->setViewAngle(angle);
            else
                error(INVALID_VALUE_FOR, "angle");
            break;
        case _ASPECT:
            advance();
            if (auto aspect = matchFloat(); aspect > 0)
                camera->setAspectRatio(aspect);
            else
                error(INVALID_VALUE_FOR, "aspect");
            break;
        case _DEPTH:
            advance();
            if (auto z = matchVec2(); z.x > 0 && z.y > z.x)
                camera->setClippingPlanes(z.x, z.y);
            else
                error(INVALID_VALUE_FOR, "depth");
            break;
        case _HEIGHT:
            advance();
            if (auto height = matchFloat(); height > 0)
                camera->setHeight(height);
            else
                error(INVALID_VALUE_FOR, "height");
            break;
        default:
            matchEndOfBlock();
            graph::CameraProxy::setCurrent(camera);
            return proxy;
        }
    }
}

inline Ref<graph::LightProxy>
SceneReaderExt::Parser::matchLight()
{
    // _COMPONENT (_LIGHT)
    advance();
    match('{');

    Ref<graph::LightProxy> proxy{graph::LightProxy::New()};

    for (auto light = proxy->light();;)
    {
        if (_token == _POINT)
        {
            advance();
            light->setType(Light::Type::Point);
        }
        else if (_token == _DIRECTIONAL)
        {
            advance();
            light->setType(Light::Type::Directional);
        }
        else if (_token == _SPOT)
        {
            advance();
            light->setType(Light::Type::Spot);
        }
        switch (_token)
        {
        case _COLOR:
            advance();
            light->color = matchColor();
            break;
        case _RANGE:
            advance();
            if (auto range = matchFloat(); range >= 0)
                light->setRange(range);
            else
                error(INVALID_VALUE_FOR, "range");
            break;
        case _ANGLE:
            advance();
            if (auto angle = matchFloat(); angle > 0)
                light->setSpotAngle(angle);
            else
                error(INVALID_VALUE_FOR, "angle");
            break;
        case _FALLOFF:
            advance();
            light->falloff = (Light::Falloff)matchIndex(0, 2);
            break;
        default:
            matchEndOfBlock();
            return proxy;
        }
    }
}

inline Ref<graph::PrimitiveProxy>
SceneReaderExt::Parser::matchPrimitive(int type)
{
    switch (type)
    {
    case _MESH:
    {
        Ref<graph::PrimitiveProxy> proxy {};

        advance();
        auto name = matchString();
        auto it = _reader->meshes.find(name);

        if (it == _reader->meshes.end())
            error(COULD_NOT_FIND_MESH, name.c_str());

        auto mesh = it->second;
        proxy = makePrimitive(*mesh, name);

        if (_token == _MATERIAL)
        {
            advance();
            auto [_, material] = *matchMaterial();
            proxy->mapper()->primitive()->setMaterial(material);
        }
        // else if (_token == _PBRMATERIAL)
        // {
        //     // not supported
        // }

        return proxy;
    } break;
    case _SURFACE:
    {
        Ref<SurfaceProxy> proxy {};

        advance();
        auto [name, surface] = *matchSurface();
        Ref<SurfacePrimitive> primitive = new SurfacePrimitive(
            dynamic_cast<GLBezierSurface*>(surface.get()));

        if (_token == _MATERIAL)
        {
            advance();
            auto [_, material] = *matchMaterial();
            primitive->setMaterial(material);
        }
        else if (_token ==_PBRMATERIAL)
        {
            advance();
            auto [_, material] = *matchPBRMaterial();
            primitive->pbrMaterial = material;
        }

        proxy = new SurfaceProxy(primitive);
        return proxy;
    } break;
    }
    return nullptr;
}

Ref<graph::Component>
SceneReaderExt::Parser::parseComponent(int type, graph::SceneObject& )
{
    return (graph::Component*)matchPrimitive(type);
}

void
SceneReaderExt::Parser::parseComponent(graph::SceneObject& object)
{
    Ref<graph::Component> component;
    auto type = (int)(size_t)_tokenValue.object;

    if (type == _CAMERA)
        component = matchCamera();
    else if (type == _LIGHT)
        component = matchLight();
    else
    {
        auto string = _lexeme;

        if ((component = parseComponent(type, object)) == nullptr)
            error(COULD_NOT_PARSE_COMPONENT, string.c_str());
    }

    auto typeName = component->typeName();

    if (object.addComponent<graph::Component>(component) == nullptr)
        error(COMPONENT_ALREADY_DEFINED, typeName);
}

} // namespace cg::util
