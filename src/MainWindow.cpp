#include "MainWindow.h"

#include <graphics/Application.h>
#include <graphics/Assets.h>
#include <graph/SceneObject.h>
#include <graph/SceneNode.h>
#include "BezierPatches.h"
#include "Surface.h"

namespace cg
{

void MainWindow::beginInitialize()
{
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    Assets::initialize();
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

    auto pipeline = new SurfacePipeline(0, *editor()->fragmentShader());
    editor()->setPipeline(GLRenderer::PipelineCode::Surface, pipeline);
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
