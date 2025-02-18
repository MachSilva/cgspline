#include "SceneReader.h"

#include <graphics/Assets.h>
#include <fstream>

namespace cg
{

using Dict = SceneReader::Dict;
// using List = SceneReader::List;
using Value = SceneReader::Value;
using enum Value::Type;

using util::_f;

SceneReader::SceneReader()
{
    _sceneComplete = false;
    _objectId = 0;
    _materialId = 0;
    _meshId = 0;
    _surfaceId = 0;

    _assets = new Dict();

    auto& s = *_scope;
    s["Assets"]      = _assets;
    s["Get"]         = std::bind_front(&SceneReader::getAsset, this);
    s["Material"]    = std::bind_front(&SceneReader::createMaterial, this);
    s["Mesh"]        = std::bind_front(&SceneReader::createMesh, this);
    s["Surface"]     = std::bind_front(&SceneReader::createSurface, this);
    s["Texture"]     = std::bind_front(&SceneReader::createTexture, this);
    s["Scene"]       = std::bind_front(&SceneReader::readScene, this);

    for (auto& kv : Assets::materials())
        _assets->insert(kv);
    
    for (auto& kv : Assets::meshes())
        _assets->insert(kv);
    
    _assets->insert({"Box", GLGraphics3::box()});
}

void SceneReader::doParse()
{
    util::DescriptionLanguage::doParse();
}

Value SceneReader::getAsset(const List& args)
{
    return _assets->at(std::get<std::string>(args.at(0)));
}

Value SceneReader::createMaterial(const List& args)
{
    auto props = args.at(0).castTo<Dict>();
    auto m = Ref(new Material(Color::black));

    if (auto p = props->get_ptr("name"))
    {
        auto& s = std::get<std::string>(*p);
        m->setName("%s", s.c_str());
    }
    else
    {
        m->setName("Material_%d", ++_materialId);
    }

    insertAsset(m->name(), m);

    if (auto p = props->get_ptr("ambient"))
    {
        m->ambient = p->getColor();
    }
    if (auto p = props->get_ptr("diffuse"))
    {
        m->diffuse = p->getColor();
    }
    if (auto p = props->get_ptr("specular"))
    {
        m->specular = p->getColor();
    }
    if (auto p = props->get_ptr("transparency"))
    {
        m->transparency = p->getColor();
    }
    if (auto p = props->get_ptr("refractive_index"))
    {
        auto f = p->getFloat();
        if (f <= 0)
            throw std::out_of_range("Material: 'refractive_index' (index of refraction) should be greater than zero");
        m->ior = f;
    }
    if (auto p = props->get_ptr("metalness"))
    {
        auto v = p->getFloat();
        if (v < 0 || v > 1)
            throw std::out_of_range("Material: 'metalness' should be between 0 and 1");
        m->metalness = v;
    }
    if (auto p = props->get_ptr("roughness"))
    {
        auto v = p->getFloat();
        if (v < 0 || v > 1)
            throw std::out_of_range("Material: 'roughness' should be between 0 and 1");
        m->roughness = v;
    }
    if (auto p = props->get_ptr("texture"))
    {
        m->texBaseColor = p->castTo<gl::Texture>()->handle();
    }
    if (auto p = props->get_ptr("metal_rough_texture"))
    {
        m->texMetalRough = p->castTo<gl::Texture>()->handle();
    }

    return m;
}

Value SceneReader::createTexture(const List& args)
{
    auto& name = std::get<std::string>(args.at(0));
    auto& path = std::get<std::string>(args.at(1));
    insertAsset(name, nullptr);

    auto file = this->workingDir() / path;
    if (std::filesystem::exists(file) == false)
        throw std::runtime_error(_f("file '{}' does not exist", path));

    return (*_assets)[name] = gl::Texture::from(file.string().c_str());
}

Value SceneReader::createMesh(const List& args)
{
    auto props = args.at(0).castTo<Dict>();

    Dict::iterator it;
    if (auto p = props->get_ptr("name"))
    {
        it = insertAsset(std::get<std::string>(*p), nullptr);
    }
    else
    {
        it = insertAsset(_f("Mesh_{}", ++_meshId), nullptr);
    }

    if (auto p = props->get_ptr("file"))
    {
        throw std::runtime_error("not yet implemented");
    }

    int vertexCount = 0;
    std::unique_ptr<vec3f[]> vertices;
    if (auto p = props->get_ptr("vertices"))
    {
        auto list = p->castTo<List>();
        vertexCount = list->size() / 3;
        vertices = std::make_unique_for_overwrite<vec3f[]>(vertexCount);

        for (auto i = 0u, j = 0u; i < vertexCount; i++, j+=3)
        {
            vertices[i].x = (*list)[j+0].getFloat();
            vertices[i].y = (*list)[j+1].getFloat();
            vertices[i].z = (*list)[j+2].getFloat();
        }
    }

    std::unique_ptr<vec3f[]> normals;
    if (auto p = props->get_ptr("normals"))
    {
        auto list = p->castTo<List>();
        if (vertexCount != list->size() / 3)
            throw std::runtime_error
                ("`normals` size must match `vertices` size");

        normals = std::make_unique_for_overwrite<vec3f[]>(vertexCount);

        for (auto i = 0u, j = 0u; i < vertexCount; i++, j+=3)
        {
            normals[i].x = (*list)[j+0].getFloat();
            normals[i].y = (*list)[j+1].getFloat();
            normals[i].z = (*list)[j+2].getFloat();
        }
    }

    int triangleCount = 0;
    std::unique_ptr<TriangleMesh::Triangle[]> triangles;
    if (auto p = props->get_ptr("triangles"))
    {
        auto list = p->castTo<List>();
        triangleCount = list->size() / 3;
        triangles = std::make_unique_for_overwrite<TriangleMesh::Triangle[]>
            (triangleCount);

        for (auto i = 0u, j = 0u; i < triangleCount; i++, j+=3)
        {
            triangles[i].v[0] = std::get<int>((*list)[j+0]);
            triangles[i].v[1] = std::get<int>((*list)[j+1]);
            triangles[i].v[2] = std::get<int>((*list)[j+2]);
        }
    }

    auto mesh = new TriangleMesh({
        .vertexCount = vertexCount,
        .triangleCount = triangleCount,
        .vertices = vertices.release(),
        .vertexNormals = normals.release(),
        .triangles = triangles.release(),
    });
    if (!mesh->hasVertexNormals())
        mesh->computeNormals();
    return it->second = mesh;
}

Value SceneReader::createSurface(const List& args)
{
    auto props = args.at(0).castTo<Dict>();

    Dict::iterator it;
    if (auto p = props->get_ptr("name"))
    {
        it = insertAsset(std::get<std::string>(*p), nullptr);
    }
    else
    {
        it = insertAsset(_f("Surface_{}", ++_surfaceId), nullptr);
    }

    if (auto p = props->get_ptr("file"))
    {
        auto& path = std::get<std::string>(*p);
        auto file = this->workingDir() / path;
        if (std::filesystem::exists(file) == false)
            throw std::runtime_error(_f("file '{}' does not exist", path));

        // std::ifstream in (file.string());
        auto s = file.string();
        return it->second = GLSurface::load(s.c_str());
    }

    throw std::runtime_error("surface data or source not provided");
}

Ref<graph::PrimitiveProxy> SceneReader::addMesh(const Dict* props)
{
    const auto& name = std::get<std::string>(props->at("name"));

    const TriangleMesh* mesh {};
    if (auto p = props->get_ptr("model"))
        mesh = p->getSharedObject<TriangleMesh>();
    if (!mesh)
        mesh = _assets->at(name).castTo<TriangleMesh>();

    auto proxy = Ref(graph::TriangleMeshProxy::New(*mesh, name));

    if (auto p = props->get_ptr("material"))
    {
        if (auto m = p->castTo<Material>())
            proxy->mapper()->primitive()->setMaterial(m);
    }

    return proxy;
}

Ref<SurfaceProxy> SceneReader::addSurface(const Dict* props)
{
    auto surface = props->at("model").castTo<GLSurface>();
    auto primitive = Ref(new SurfacePrimitive(surface));

    if (auto p = props->get_ptr("material"))
    {
        if (auto m = p->getSharedObject<Material>())
            primitive->setMaterial(m);
        else
            throw std::runtime_error("invalid material type for surface");
    }

    return new SurfaceProxy(primitive);
}

static Ref<graph::CameraProxy> createCamera(const Dict* props)
{
    auto proxy = Ref(graph::CameraProxy::New());
    auto camera = proxy->camera();

    if (auto p = props->get_ptr("angle"))
    {
        if (auto f = p->getFloat(); f > 0)
            camera->setViewAngle(f);
        else
            throw std::out_of_range("Camera: angle property must be positive");
    }
    if (auto p = props->get_ptr("aspect"))
    {
        if (auto f = p->getFloat(); f > 0)
            camera->setAspectRatio(f);
        else
            throw std::out_of_range("Camera: aspect property must be positive");
    }
    if (auto p = props->get_ptr("depth"))
    {
        if (auto f = std::get<vec2f>(*p); f.x > 0 && f.y > f.x)
            camera->setClippingPlanes(f.x, f.y);
        else
            throw std::out_of_range("Camera: invalid depth value");
    }
    if (auto p = props->get_ptr("height"))
    {
        if (auto f = p->getFloat(); f > 0)
            camera->setHeight(f);
        else
            throw std::out_of_range("Camera: height property must be positive");
    }
    if (auto p = props->get_ptr("projection"))
    {
        auto& s = std::get<std::string>(*p);
        if (s == "perspective")
            camera->setProjectionType(Camera::Perspective);
        else if (s == "parallel")
            camera->setProjectionType(Camera::Parallel);
        else
            throw std::out_of_range("Camera: projection must be perspective or parallel");
    }

    return proxy;
}

static Ref<graph::LightProxy> createLight(const Dict* props)
{
    auto proxy = Ref(graph::LightProxy::New());
    auto light = proxy->light();
    light->falloff = Light::Falloff::Quadratic; // quadratic falloff by default

    if (auto p = props->get_ptr("type"))
    {
        auto& s = std::get<std::string>(*p);

        if (s == "point")
            light->setType(Light::Type::Point);
        else if (s == "directional")
            light->setType(Light::Type::Directional);
        else if (s == "spot")
            light->setType(Light::Type::Spot);
        else
            throw std::out_of_range("Light: type must be one of: {point, directional, spot}");
    }
    if (auto p = props->get_ptr("angle"))
    {
        if (auto f = p->getFloat(); f > 0)
            light->setSpotAngle(f);
        else
            throw std::out_of_range("Light: angle property must be positive");
    }
    if (auto p = props->get_ptr("color"))
    {
        light->color = p->getColor();
    }
    if (auto p = props->get_ptr("range"))
    {
        if (auto f = p->getFloat(); f > 0)
            light->setRange(f);
        else
            throw std::out_of_range("Light: range property must be positive");
    }
    if (auto p = props->get_ptr("falloff"))
    {
        if (auto f = std::get<int>(*p); f >= 0 && f <= 2)
            light->falloff = (Light::Falloff) f;
        else
            throw std::out_of_range("Light: falloff property must be an integer in [0,2]");
    }

    return proxy;
}

static void parseTransform(graph::SceneObject* obj, const Dict* props)
{
    auto t = obj->transform();
    if (auto p = props->get_ptr("position"))
        t->setLocalPosition(std::get<vec3f>(*p));
    if (auto p = props->get_ptr("rotation"))
        t->setLocalEulerAngles(std::get<vec3f>(*p));
    if (auto p = props->get_ptr("scale"))
        t->setLocalScale(std::get<vec3f>(*p));
}

void SceneReader::createSceneObject(const Dict* props, graph::SceneObject* parent)
{
    auto obj = graph::SceneObject::New(*_scene);

    if (auto p = props->get_ptr("name"))
        obj->setName("%s", std::get<std::string>(*p).c_str());
    else
        obj->setName("Object %d", ++_objectId);

    obj->setParent(parent);

    if (auto p = props->get_ptr("light"))
        obj->addComponent(createLight(p->castTo<Dict>()).get());

    if (auto p = props->get_ptr("mesh"))
        obj->addComponent(addMesh(p->castTo<Dict>()).get());
    
    if (auto p = props->get_ptr("surface"))
        obj->addComponent(addSurface(p->castTo<Dict>()).get());

    if (auto p = props->get_ptr("objects"))
    {
        auto list = p->castTo<List>();
        for (const auto& e : *list)
        {
            createSceneObject(e.castTo<Dict>(), obj);
        }
    }

    if (auto p = props->get_ptr("transform"))
        parseTransform(obj, p->castTo<Dict>());
}

Value SceneReader::readScene(const List& args)
{
    auto props = args.at(0).castTo<Dict>();

    _scene = graph::Scene::New();

    if (auto p = props->get_ptr("environment"))
    {
        auto d = p->castTo<Dict>();
        if (auto q = d->get_ptr("ambient"))
            _scene->ambientLight = q->getColor();
        if (auto q = d->get_ptr("background"))
            _scene->backgroundColor = q->getColor();
        if (auto q = d->get_ptr("texture"))
            environment = q->castTo<gl::Texture>();
    }

    if (auto p = props->get_ptr("view"))
    {
        auto d = p->castTo<Dict>();
        if (auto q = d->get_ptr("position"))
            view.position = std::get<vec3f>(*q);
        if (auto q = d->get_ptr("rotation"))
            view.rotation = quatf::eulerAngles(std::get<vec3f>(*q));
    }

    if (auto p = props->get_ptr("objects"))
    {
        auto list = p->castTo<List>();
        auto root = _scene->root();
        for (const auto& e : *list)
        {
            createSceneObject(e.castTo<Dict>(), root);
        }
    }
    return _scene;
}

} // namespace cg
