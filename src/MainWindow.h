#pragma once

#include <filesystem>
#include <future>
#include <memory_resource>
#include <vector>
#include <CUDAHelper.h>
#include <graph/SceneObject.h>
#include <graph/SceneWindow.h>
#include <graphics/GLImage.h>
#include <graphics/Renderer.h>
#include "GLBezierSurface.h"
#include "Framebuffer.h"
#include "SceneReaderExt.h"
#include "Spline.h"
#include "SplRenderer.h"
#include "Surface.h"
#include "RayTracer.h"
#include "rt/CPURayTracer.h"
#include "rt/RayTracer.h"

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
    graph::SceneObject* createSurfaceObject(GLBezierSurface& p, const char* name);
    void drawSelectedObject(const graph::SceneObject& object);

    graph::SceneObject* pickObject(graph::SceneObject* obj,
        const Ray3f& ray,
        Intersection& hit) const;
    
    void readScene(std::filesystem::path scenefile);
    void convertScene();

    void assetWindow();
    void cameraPreview();
    void controlWindow();
    void fileMenu();
    void mainMenu();
    void openSceneMenu(std::string_view label);
    void viewMenu();
    void helpMenu();
    static void inspectSurface(MainWindow&, SurfaceProxy&);

    void prevRenderScene();
    Ref<GLImage> _prevImage;
    Ref<RayTracer> _prevRayTracer;

    Intersection _lastPickHit;
    Ref<gl::Texture> _image;
    Ref<rt::Frame> _frame;
    Ref<rt::CPURayTracer> _rayTracer;
    std::unique_ptr<rt::Scene> _rtScene;
    rt::Camera _rtCamera;
    Ref<SplRenderer> _texRenderer;

    rt::ManagedCUDAResource _cudaManaged;

    std::vector<std::filesystem::path> _sceneFileList;
    MaterialMap _sceneMaterials;
    util::TriangleMeshMap _sceneMeshes;
    util::PBRMaterialMap _scenePBRMaterials;
    util::SurfaceMap _sceneSurfaces;
    util::TextureMap _sceneTextures;
    Ref<gl::Texture> _sceneEnvironment;

    State _state {};
    Data _data {};

    struct
    {
      int device = -1;
      cudaDeviceProp properties;
    } _cuda;

#if SPL_BC_STATS
    Ref<graph::SceneObject> _debugObject;
    Ref<GLBezierSurface> _debugPatch2D;
    uint32_t _debugPatchIndex {};
    uint32_t _debugStep {};
#endif
};

} // namespace cg
