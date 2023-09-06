#include "MainWindow.h"

#include <graphics/Application.h>
#include <graphics/Assets.h>
#include <graph/SceneObject.h>
#include <graph/SceneNode.h>
#include <format>
#include <map>
#include <ranges>
#include "SceneReaderExt.h"

namespace cg
{

#ifndef NDEBUG
static
void debugCallback(GLenum source,
    GLenum type,
    GLuint id,
    GLenum severity,
    GLsizei length,
    const GLchar* message,
    const void* userParam)
{
    static std::map<GLuint, const char*> msg {
        {GL_DEBUG_TYPE_ERROR,               "[  ERROR  ]"},
        {GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR, "DEPRECATED_BEHAVIOR"},
        {GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR,  "UNDEFINED_BEHAVIOR"},
        {GL_DEBUG_TYPE_PORTABILITY,         "PORTABILITY"},
        {GL_DEBUG_TYPE_PERFORMANCE,         "PERFORMANCE"},
        {GL_DEBUG_TYPE_OTHER,               "OTHER"},
        {GL_DEBUG_TYPE_MARKER,              "MARKER"},
        {GL_DEBUG_TYPE_PUSH_GROUP,          "PUSH_GROUP"},
        {GL_DEBUG_TYPE_POP_GROUP,           "POP_GROUP"},
        {GL_DEBUG_SEVERITY_HIGH,            "HIGH"},
        {GL_DEBUG_SEVERITY_MEDIUM,          "MEDIUM"},
        {GL_DEBUG_SEVERITY_LOW,             "LOW"},
        {GL_DEBUG_SEVERITY_NOTIFICATION,    "NOTIFICATION"},
    };

    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) return;
    if (severity == GL_DEBUG_SEVERITY_LOW) return;
    //if (severity == GL_DEBUG_SEVERITY_MEDIUM) return;
    //if (severity == GL_DEBUG_SEVERITY_HIGH) return;

    fprintf(stderr, "GL DEBUG (type=%s, severity=%s): %s",
        msg[type], msg[severity], message);
    fprintf(stderr, "\n");
}
#endif

void MainWindow::beginInitialize()
{
    const auto vendor   = glGetString(GL_VENDOR);
    const auto renderer = glGetString(GL_RENDERER);
    GLint version[2];

    glGetIntegerv(GL_MAJOR_VERSION, version);
    glGetIntegerv(GL_MINOR_VERSION, version+1);
    
    std::cerr << "CG Spline Project\n"
        << "OpenGL version " << version[0] << "." << version[1] << "\n"
        << "Renderer: " << renderer << "\n"
        << "Vendor:   " << vendor << "\n\n";

    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    Assets::initialize();

#ifndef NDEBUG
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(debugCallback, this);
#endif

    _sceneFileList.clear();
    auto sceneDir = Application::assetFilePath("scenes");
    if (std::filesystem::exists(sceneDir))
        for (auto entry : std::filesystem::directory_iterator(sceneDir))
        {
            if (entry.is_regular_file())
                _sceneFileList.emplace_back(entry.path());
        }
}

void MainWindow::initializeScene()
{
    float aspect = float(width()) / height();
    auto obj = createCameraObject(aspect, "Main Camera");
    obj->transform()->setLocalPosition({0,0,10});

    obj = createLightObject(Light::Type::Directional, "The Light");
    obj->transform()->setLocalEulerAngles({50,130,0});

    createPrimitiveObject(*GLGraphics3::box(), "Box");

    // Load surfaces
    std::tuple<const char*, vec3f, vec3f, float> surfaces[] = {
        {"bezier/teacup", vec3f(0, 0, 0), vec3f::null(), 1.0},
        {"bezier/teapot", vec3f(-2.5, 0, 0), vec3f(-90, 0, 0), 0.5},
        {"bezier/teaspoon", vec3f(1.5, 0, 0), vec3f::null(), 1.0}
    };

    for (auto& [s, t, r, sc] : surfaces)
    {
        auto asset = Application::assetFilePath(s);
        auto p = BezierPatches::load(asset.c_str());
        auto transform = createSurfaceObject(*p, s)->transform();
        transform->translate(t);
        transform->rotate(r);
        transform->setLocalScale(sc);
    }

    editor()->setPipeline(
        GLRenderer::Surface,
        new SurfacePipeline(0, *editor()->fragmentShader())
    );
    editor()->setPipeline(
        "SurfCont"_ID8,
        new SurfaceContourPipeline(0, *editor()->fragmentShader())
    );

    registerInspectFunction(inspectSurface);
}

void MainWindow::render()
{
    if (_viewMode != ViewMode::Editor)
    {
        renderScene();
        return;
    }

    auto e = editor();
    e->render();
    e->drawXZPlane(10, 1);

    // Draw selected object
    if (auto obj = currentNode()->as<graph::SceneObject>())
    {
        const auto t = obj->transform();
        drawSelectedObject(*obj);
        if (_cursorMode == CursorMode::Select)
            editor()->drawTransform(t->position(), t->rotation());

        const SurfaceProxy* p = nullptr;
        for (graph::Component* c : obj->components())
            if (auto ptr = dynamic_cast<const SurfaceProxy*>(c))
            {
                p = ptr;
                break;
            }

        if (_cursorMode == CursorMode::NormalInspect
            && _lastPickHit.object == obj
            && p != nullptr)
        {
            Surface& s = p->mapper()->surface();
            vec3f n = s.normal(_lastPickHit);
            vec4f P = s.point(_lastPickHit);
            vec3f Q = vec3f(P.x / P.w, P.y / P.w, P.z / P.w);
            auto e = editor();
            e->setVectorColor(Color{0x9c, 0x27, 0xb0}); // Purple A500
            e->setBasePoint(Q);
            e->drawVector(Q, n, e->pixelsLength(100.0));
        }
    }
}

static volatile int _progress[2];

static
void progressCallback(int i, int n)
{
    _progress[0] = i;
    _progress[1] = n;
}

void MainWindow::renderScene()
{
    if (_viewMode != ViewMode::Renderer)
        return;
    
    auto camera = graph::CameraProxy::current();
    if (!camera)
        camera = editor()->camera();
    
    if (_image == nullptr)
    {
        // Create task
        // if (!_renderTask.valid()) {}
        _image = new GLImage(width(), height());
        if (_rayTracer == nullptr)
            _rayTracer = new RayTracer(*scene(), *camera);
        else
            _rayTracer->setCamera(*camera);
        // _rayTracer->setProgressCallback(progressCallback);
        _rayTracer->setMaxRecursionLevel(6);
        _rayTracer->renderImage(*_image);
    }
    _image->draw(0, 0);
}

void MainWindow::drawSelectedObject(const graph::SceneObject& object)
{
    if (!object.visible()) return;

    for (auto component : object.components())
    {
        if (auto s = dynamic_cast<const SurfaceProxy*>(component))
            s->mapper()->renderContour(*editor());
    }

    SceneWindow::drawComponents(object);
    
    for (auto& child : object.children())
        drawSelectedObject(*child);
}

void MainWindow::gui()
{
    mainMenu();
    assetWindow();
    hierarchyWindow();
    inspectorWindow();
    editorView();
    controlWindow();
}

inline
void MainWindow::fileMenu()
{
    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("New Scene"))
            newScene();
        openSceneMenu("Open");
        ImGui::Separator();
        if (ImGui::MenuItem("Exit", "Alt+F4"))
            shutdown();

        ImGui::EndMenu();
    }
}

inline
void MainWindow::openSceneMenu(std::string_view label)
{
    if (ImGui::BeginMenu("Open", !_sceneFileList.empty()))
    {
        for (auto& file : _sceneFileList)
            if (ImGui::MenuItem(file.filename().c_str()))
                readScene(file);
        ImGui::EndMenu();
    }
}

inline
void MainWindow::helpMenu()
{
    if (ImGui::BeginMenu("Help"))
    {
        if (ImGui::MenuItem("Keybindings"))
            0;
        ImGui::EndMenu();
    }
}

inline
void MainWindow::mainMenu()
{
    if (ImGui::BeginMainMenuBar())
    {
        fileMenu();
        helpMenu();
        ImGui::EndMainMenuBar();
    }
}

void MainWindow::controlWindow()
{
    ImGui::Begin("Control Window");

    if (_image != nullptr)
        ImGui::Text("Image of (%d, %d).", _image->width(), _image->height());
    else
        ImGui::Text("No image.");

    if (ImGui::Button("Render"))
    {
        _viewMode = ViewMode::Renderer;
        _image = nullptr;
    }
    ImGui::SameLine();

    ImGui::BeginDisabled(_image == nullptr);
    if (ImGui::Button("Show/Hide"))
    {
        if (_viewMode != ViewMode::Renderer)
            _viewMode = ViewMode::Renderer;
        else
            _viewMode = ViewMode::Editor;
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (ImGui::Button("Clear"))
    {
        _viewMode = ViewMode::Editor;
        _image = nullptr;
    }

    ImGui::Separator();

    const char* items[] = {
        "Select",
        "Primitive Inspect",
        "Normal Inspect"
    };
    CursorMode modes[] = {
        Select,
        PrimitiveInspect,
        NormalInspect,
    };
    int mode = _cursorMode;
    if (ImGui::Combo("Cursor Mode", &mode, items, std::size(items)))
        _cursorMode = modes[mode];

    ImGui::End();
}

void MainWindow::inspectSurface(MainWindow& window, SurfaceProxy& s)
{
    ImGui::inputText("Surface", s.sceneObject()->name());

    window.inspectMaterial(s.mapper()->surface());
}

bool MainWindow::onKeyPress(int key, int p2)
{
    if (_viewMode == ViewMode::Editor)
    {
        switch (key)
        {
        case GLFW_KEY_F1: _cursorMode = CursorMode::Select; break;
        case GLFW_KEY_F2: _cursorMode = CursorMode::PrimitiveInspect; break;
        case GLFW_KEY_F3: _cursorMode = CursorMode::NormalInspect; break;
        }
    }

    return SceneWindow::onKeyPress(key, p2);
}

bool MainWindow::onMouseLeftPress(int x, int y)
{
    auto ray = makeRay(x, y);
    _lastPickHit.distance = math::Limits<float>::inf();
    _lastPickHit.object = nullptr;
    auto obj = pickObject(_scene->root(), ray, _lastPickHit);
    auto current = this->currentNode();
    _lastPickHit.object = obj;

    if (obj && obj->selectable())
    {
        if (auto p = current->as<graph::SceneObject>())
            p->setSelected(false);
        obj->setSelected(true);
        *current = obj;
    }

    return true;
}

graph::SceneObject *MainWindow::pickObject(int x, int y) const
{
    auto ray = makeRay(x, y);
    Intersection hit;
    hit.distance = math::Limits<float>::inf();
    hit.object = nullptr;
    return pickObject(_scene->root(), ray, hit);
}

graph::SceneObject* MainWindow::pickObject(graph::SceneObject *obj,
    const Ray3f &ray,
    Intersection &hit) const
{
    if (!obj->visible()) return nullptr;

    graph::SceneObject* nearest = nullptr;

    for (graph::Component* component : obj->components())
    {
        // XXX This should be optimized out.
        Intersection subhit;
        subhit.distance = math::Limits<float>::inf();
        subhit.object = nullptr;

        if (auto p = dynamic_cast<graph::PrimitiveProxy*>(component))
        {
            auto primitive = p->mapper()->primitive();
            if (primitive->intersect(ray, subhit) == false)
                continue;
        }
        else if (auto s = dynamic_cast<SurfaceProxy*>(component))
        {
            auto surface = s->mapper()->surface();
            if (surface.intersect(ray, subhit) == false)
                continue;
        }
        else continue; // no component => no hit => next iteration

        // In case of a hit
        if (subhit.distance < hit.distance)
        {
            hit = subhit;
            nearest = obj;
        }
    }

    for (auto& child : obj->children())
        if (auto tmp = pickObject(&child, ray, hit))
            nearest = tmp;

    return nearest;
}

void MainWindow::readScene(std::filesystem::path scenefile)
{
    util::SceneReaderExt reader {};

    reader.setInput(scenefile.c_str());
    try
    {
        reader.execute();
        if (reader.scene() == nullptr)
            throw std::runtime_error("Scene is null");
        setScene(*reader.scene());

        auto& materials = Assets::materials();
        for (auto& [name, m] : reader.materials)
            materials[name] = m;
    }
    catch (const std::exception& e)
    {
        std::cerr << std::format("Failed to load scene '{}'. Reason:\n{}\n",
            scenefile.filename().c_str(), e.what());
    }
}

graph::SceneObject*
MainWindow::createSurfaceObject(BezierPatches &p, const char *name)
{
    auto object = graph::SceneObject::New(*_scene);

    object->setName("%s", name);
    object->addComponent(new SurfaceProxy(&p));

    return object;
}

} // namespace cg
