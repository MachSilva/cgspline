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

    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(debugCallback, this);
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
    const char* surfaces[] = {
        "bezier/teacup",
        "bezier/teapot",
        "bezier/teaspoon"
    };

    for (auto& s : surfaces)
    {
        auto asset = Application::assetFilePath(s);
        auto p = BezierPatches::load(asset.c_str());
        createSurfaceObject(*p, s);
    }

    //auto pipeline = new SurfacePipeline(0, *editor()->fragmentShader());
    //editor()->setPipeline(GLRenderer::PipelineCode::Surface, pipeline);
}

void MainWindow::render()
{
    auto e = editor();
    e->render();
    e->drawXZPlane(10, 1);

    // auto scene = e->scene();
    // scene->lights();

    if (auto obj = currentNode()->as<graph::SceneObject>())
    {
        auto t = obj->transform();
        drawSelectedObject(*obj);
        editor()->drawTransform(t->position(), t->rotation());
    }
}

void MainWindow::gui()
{
    assetWindow();
    hierarchyWindow();
    inspectorWindow();
    editorView();
}

graph::SceneObject* MainWindow::createSurfaceObject(BezierPatches &p, const char *name)
{
    auto object = graph::SceneObject::New(*_scene);

    object->setName("%s", name);
    object->addComponent(new SurfaceProxy(&p));

    return object;
}

} // namespace cg
