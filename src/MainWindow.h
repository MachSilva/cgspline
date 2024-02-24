#pragma once

#include <filesystem>
#include <future>
#include <vector>
#include <graph/SceneWindow.h>
#include <graph/SceneObject.h>
#include <graphics/GLImage.h>
#include <graphics/Renderer.h>
#include "BezierPatches.h"
#include "Framebuffer.h"
#include "RayTracer.h"
#include "SceneReaderExt.h"
#include "Surface.h"
#include "Spline.h"

namespace cg
{

class MainWindow : public graph::SceneWindow
{
    using Base = graph::SceneWindow;

public:
    MainWindow(int width, int height)
        : SceneWindow("CG::SPLINE", width, height) {}

    void beginInitialize() override;
    void initializeScene() override;
    void render() override;
    void renderScene() override;
    void gui() override;
    bool onResize(int width, int height) override;
    bool onKeyPress(int key, int) override;
    bool onMouseLeftPress(int, int) override;
    graph::SceneObject* pickObject(int x, int y) const override;

    enum CursorMode
    {
        Select = 0,
        PrimitiveInspect,
        NormalInspect,
    };

    enum ShadingMode
    {
        CookTorrance = 0,
        Phong,
    };

    struct State
    {
        bool renderOnCentralNode = true;
        CursorMode cursorMode = CursorMode::Select;
        ShadingMode shadingMode = ShadingMode::CookTorrance;
        Ref<gl::Framebuffer> renderFramebuffer;
        Ref<gl::Framebuffer> cameraFramebuffer;
        Viewport renderViewport;
        Viewport cameraViewport;
    };

    struct Data
    {
        GLint windowColorEncoding;
        GLint windowComponentType;
        GLint windowRedBits;
        GLint windowGreenBits;
        GLint windowBlueBits;
        GLint windowSamples;
        GLint windowSampleBuffers;
    };

protected:
    graph::SceneObject* createSurfaceObject(BezierPatches& p, const char* name);
    void drawSelectedObject(const graph::SceneObject& object);

    graph::SceneObject* pickObject(graph::SceneObject* obj,
        const Ray3f& ray,
        Intersection& hit) const;
    
    void readScene(std::filesystem::path scenefile);

    void assetWindow();
    void cameraPreview();
    void controlWindow();
    void fileMenu();
    void mainMenu();
    void openSceneMenu(std::string_view label);
    void viewMenu();
    void helpMenu();
    static void inspectSurface(MainWindow&, SurfaceProxy&);

    std::future<void> _renderTask;
    Ref<GLImage> _image;
    Ref<RayTracer> _rayTracer;
    Intersection _lastPickHit;

    std::vector<std::filesystem::path> _sceneFileList;
    MaterialMap _sceneMaterials;
    util::TriangleMeshMap _sceneMeshes;
    util::PBRMaterialMap _scenePBRMaterials;
    util::SurfaceMap _sceneSurfaces;
    util::TextureMap _sceneTextures;
    Ref<gl::Texture> _sceneEnvironment;

    State _state {};
    Data _data {};

    Ref<graph::SceneObject> _debugObject;
    Ref<BezierPatches> _debugPatch2D;
    uint32_t _debugPatchIndex {};
};

} // namespace cg
