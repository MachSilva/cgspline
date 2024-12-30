#pragma once

#include <filesystem>
#include <future>
#include <map>
#include <memory_resource>
#include <vector>
#include <CUDAHelper.h>
#include <graph/SceneObject.h>
#include <graph/SceneWindow.h>
#include <graphics/GLImage.h>
#include <graphics/Renderer.h>
#include <imgui.h>
#include "ColorMap.h"
#include "GLSurface.h"
#include "Framebuffer.h"
// #include "SceneReaderExt.h"
#include "Spline.h"
#include "SplRenderer.h"
#include "SurfacePipeline.h"
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
        // bool backgroundTask = false;
        void (MainWindow::*backgroundTaskStatusWindow)() = nullptr;
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

    // #ffffff
    // #fffb00
    // #fe2500
    // #000000
    static constexpr float heatMaxValue = 16;
    static constexpr ColorMap<4> heatPallete {{vec4f
        {0, 0, 0, 1},
        {254.f/255, 37.f/255, 0, 1},
        {1, 251.f/255, 0, 1},
        {1, 1, 1, 1}
    }, heatMaxValue};

protected:
    graph::SceneObject* createSurfaceObject(GLSurface& p, const char* name);
    void drawSelectedObject(const graph::SceneObject& object);

    graph::SceneObject* pickObject(graph::SceneObject* obj,
        const Ray3f& ray,
        Intersection& hit) const;

    void createDebugObject(const char * name = "Debug Object");

    void readScene(std::filesystem::path scenefile);
    void reloadScene();
    void setScene(graph::Scene& scene);
    void convertScene();

    static constexpr auto c_CPURayTracerStatusTitle = "CPU Ray Tracing";
    static constexpr auto c_CUDARayTracerStatusTitle = "CUDA Ray Tracing";
    static constexpr auto c_StatusWindowFlags
        = ImGuiWindowFlags_AlwaysAutoResize
        | ImGuiWindowFlags_NoDocking
        | ImGuiWindowFlags_NoCollapse
        | ImGuiWindowFlags_NoMove
        | ImGuiWindowFlags_NoSavedSettings;

    void whenCPURayTracerEnds();
    void whenCUDARayTracerEnds();

    // void backgroundTaskWindow();
    // void rayTracerStatusModal();
    void cpuRayTracerStatusWindow();
    void cudaRayTracerStatusWindow();
    void assetWindow();
    void cameraPreview();
    void controlWindow();
    void workbenchWindow();
    void fileMenu();
    void mainMenu();
    void openSceneMenu(std::string_view label);
    void viewMenu();
    void helpMenu();
    static void inspectLight(MainWindow&, graph::LightProxy&);
    static void inspectSurface(MainWindow&, SurfaceProxy&);

    void prevRenderScene();
    Ref<GLImage> _prevImage;
    Ref<RayTracer> _prevRayTracer;

    enum RenderMethod
    {
        eCPU, eCUDA
    } _renderMethod = RenderMethod::eCPU;

    std::chrono::steady_clock::time_point _lastRenderStarted;
    std::string _lastRenderInfo;
    Ref<gl::Texture> _image;
    Ref<rt::Frame> _frame;
    Ref<rt::RayTracer> _rayTracer;
    Ref<rt::CPURayTracer> _cpuRayTracer;
    rt::CPURayTracer::Options _cpuRTOptions { .threads = 4 };
    std::unique_ptr<rt::Scene> _rtScene;
    rt::Camera _rtCamera;

    Ref<SplRenderer> _texRenderer;
    std::map<std::string,Ref<gl::Texture>> _workbench2D;

    Ref<gl::Texture> _workbench2DSelectedItem = nullptr;

    std::vector<std::filesystem::path> _sceneFileList;
    std::map<std::string,Ref<Material>> _sceneMaterials;
    std::map<std::string,Ref<TriangleMesh>> _sceneMeshes;
    std::map<std::string,Ref<PBRMaterial>> _scenePBRMaterials;
    std::map<std::string,Ref<GLSurface>> _sceneSurfaces;
    std::map<std::string,Ref<gl::Texture>> _sceneTextures;
    Ref<gl::Texture> _sceneEnvironment;

    State _state {};
    Data _data {};

    struct
    {
      int device = -1;
      cudaDeviceProp properties;
    } _cuda;

    struct SceneRefs
    {
        std::filesystem::path filepath;
        Intersection lastPickHit {};
        graph::SceneObject* debugObject {};

        void reset() { *this = {}; }
    } _sceneRefs;

// #if SPL_BC_STATS
    Ref<GLSurface> _debugPatch2D;
    uint32_t _debugPatchIndex {};
    uint32_t _debugStep {};
// #endif

    vec2 _debugClip[2] {{0, 1}, {0, 1}};
    bool _matClip = false;
};

} // namespace cg
