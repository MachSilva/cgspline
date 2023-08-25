#include "MainWindow.h"

#include <graphics/Application.h>
#include <graphics/Assets.h>
#include <graph/SceneObject.h>
#include <graph/SceneNode.h>
#include <map>
#include "BezierPatches.h"
#include "Surface.h"

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
    std::pair<const char*, vec3f> surfaces[] = {
        {"bezier/teacup", vec3f(0, 0, 0)},
        {"bezier/teapot", vec3f(-2.5, 0, 0)},
        {"bezier/teaspoon", vec3f(1.5, 0, 0)}
    };

    for (auto& [s, t] : surfaces)
    {
        auto asset = Application::assetFilePath(s);
        auto p = BezierPatches::load(asset.c_str());
        createSurfaceObject(*p, s)->transform()->translate(t);
    }

    editor()->setPipeline(
        GLRenderer::Surface,
        new SurfacePipeline(0, *editor()->fragmentShader())
    );
    editor()->setPipeline(
        "SurfCont"_ID8,
        new SurfaceContourPipeline(0, *editor()->fragmentShader())
    );
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

    if (auto obj = currentNode()->as<graph::SceneObject>())
    {
        const auto t = obj->transform();
        drawSelectedObject(*obj);
        editor()->drawTransform(t->position(), t->rotation());
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
    assetWindow();
    hierarchyWindow();
    inspectorWindow();
    editorView();
    controlWindow();
}

void MainWindow::controlWindow()
{
    ImGui::Begin("Control Window");

    if (ImGui::Button("Render"))
    {
        _viewMode = ViewMode::Renderer;
        _image = nullptr;
    }
    if (ImGui::Button("Clear"))
    {
        _viewMode = ViewMode::Editor;
        _image = nullptr;
    }

    ImGui::End();
}

graph::SceneObject* MainWindow::pickObject(int x, int y) const
{
    auto ray = makeRay(x, y);
    auto distance = math::Limits<float>::inf();
    return pickObject(_scene->root(), ray, distance);
}

graph::SceneObject *MainWindow::pickObject(graph::SceneObject *obj,
    const Ray3f &ray,
    float &distance) const
{
    if (!obj->visible()) return nullptr;

    graph::SceneObject* nearest = nullptr;

    for (graph::Component* component : obj->components())
    {
        Intersection hit;
        hit.distance = math::Limits<float>::inf();
        hit.object = nullptr;

        if (auto p = dynamic_cast<graph::PrimitiveProxy*>(component))
        {
            auto primitive = p->mapper()->primitive();
            if (primitive->intersect(ray, hit) == false)
                continue;
        }
        else if (auto s = dynamic_cast<SurfaceProxy*>(component))
        {
            auto surface = s->mapper()->surface();
            if (surface.intersect(ray, hit) == false)
                continue;
        }
        else continue; // No hit => next iteration

        // In case of a hit
        if (hit.distance < distance)
        {
            distance = hit.distance;
            nearest = obj;
        }
    }

    for (auto& child : obj->children())
        if (auto tmp = pickObject(&child, ray, distance))
            nearest = tmp;

    return nearest;
}

graph::SceneObject* MainWindow::createSurfaceObject(BezierPatches &p, const char *name)
{
    auto object = graph::SceneObject::New(*_scene);

    object->setName("%s", name);
    object->addComponent(new SurfaceProxy(&p));

    return object;
}

} // namespace cg
