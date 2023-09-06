#pragma once

#include <filesystem>
#include <future>
#include <vector>
#include <graph/SceneWindow.h>
#include <graph/SceneObject.h>
#include <graphics/GLImage.h>
#include "BezierPatches.h"
#include "RayTracer.h"
#include "Surface.h"

namespace cg
{

class MainWindow : public graph::SceneWindow
{
public:
    MainWindow(int width, int height)
        : SceneWindow("CG::SPLINE", width, height) {}

    void beginInitialize() override;
    void initializeScene() override;
    void render() override;
    void renderScene() override;
    void gui() override;
    // bool onResize(int width, int height) override;
    bool onKeyPress(int key, int) override;
    bool onMouseLeftPress(int, int) override;
    graph::SceneObject* pickObject(int x, int y) const override;

    enum CursorMode
    {
        Select = 0,
        PrimitiveInspect,
        NormalInspect,
    };

    CursorMode _cursorMode = CursorMode::Select;

protected:
    graph::SceneObject* createSurfaceObject(BezierPatches& p, const char* name);
    void drawSelectedObject(const graph::SceneObject& object);

    graph::SceneObject* pickObject(graph::SceneObject* obj,
        const Ray3f& ray,
        Intersection& hit) const;
    
    void readScene(std::filesystem::path scenefile);
    
    void controlWindow();
    void fileMenu();
    void mainMenu();
    void openSceneMenu(std::string_view label);
    void helpMenu();
    static void inspectSurface(MainWindow&, SurfaceProxy&);

    std::future<void> _renderTask;
    Reference<GLImage> _image;
    Reference<RayTracer> _rayTracer;
    Intersection _lastPickHit;

    std::vector<std::filesystem::path> _sceneFileList;
};

} // namespace cg
