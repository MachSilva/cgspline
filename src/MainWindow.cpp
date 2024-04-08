#include "MainWindow.h"

#include <graphics/Application.h>
#include <graphics/Assets.h>
#include <graph/SceneObject.h>
#include <graph/SceneNode.h>
#include <format>
#include <map>
#include <ranges>
#include "Framebuffer.h"
#include "SceneReaderExt.h"

#include <imgui_internal.h>

namespace cg
{

#ifdef CG_GL_DEBUG
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

#ifdef CG_GL_DEBUG
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(debugCallback, this);
#endif

    cuda::initialize();
    _cuda.device = cuda::currentDevice();
    checkCudaError(cudaGetDeviceProperties(&_cuda.properties, _cuda.device));

    auto& p = _cuda.properties;
    std::cerr <<
        "CUDA device name:  " << p.name <<
      "\nMemory Bus Width:  " << p.memoryBusWidth << " bits"
      "\nMemory Clock Rate: " << (p.memoryClockRate / 1024.0) << " MHz"
      "\n\n";

    glEnable(GL_FRAMEBUFFER_SRGB);

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0U);
    glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER, GL_FRONT,
        GL_FRAMEBUFFER_ATTACHMENT_COLOR_ENCODING,
        &_data.windowColorEncoding);
    glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER, GL_FRONT,
        GL_FRAMEBUFFER_ATTACHMENT_COMPONENT_TYPE,
        &_data.windowComponentType);
    glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER, GL_FRONT,
        GL_FRAMEBUFFER_ATTACHMENT_RED_SIZE,
        &_data.windowRedBits);
    glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER, GL_FRONT,
        GL_FRAMEBUFFER_ATTACHMENT_GREEN_SIZE,
        &_data.windowGreenBits);
    glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER, GL_FRONT,
        GL_FRAMEBUFFER_ATTACHMENT_BLUE_SIZE,
        &_data.windowBlueBits);

    glGetFramebufferParameteriv(GL_DRAW_FRAMEBUFFER, GL_SAMPLES,
        &_data.windowSamples);
    glGetFramebufferParameteriv(GL_DRAW_FRAMEBUFFER, GL_SAMPLE_BUFFERS,
        &_data.windowSampleBuffers);

    gl::AttachmentDescription color
    {
        .attachment = GL_COLOR_ATTACHMENT0,
        .format = GL_RGBA8
    };
    gl::AttachmentDescription depthStencil
    {
        .attachment = GL_DEPTH_STENCIL_ATTACHMENT,
        .format = GL_DEPTH24_STENCIL8
    };
    gl::FramebufferDescription description
    {
        .width = (uint32_t) framebufferWidth,
        .height = (uint32_t) framebufferHeight,
        .samples = (uint32_t) _data.windowSamples,
        .pRenderbuffers = &depthStencil,
        .renderbufferCount = 1,
        .pTextures = &color,
        .textureCount = 1
    };

    // _state.renderFramebuffer = new gl::Framebuffer(&description);
    _state.cameraFramebuffer = new gl::Framebuffer(&description);

    _sceneFileList.clear();
    auto sceneDir = Application::assetFilePath("scenes");
    if (std::filesystem::exists(sceneDir))
        for (auto entry : std::filesystem::directory_iterator(sceneDir))
        {
            if (entry.is_regular_file())
                _sceneFileList.emplace_back(entry.path());
        }

    _sceneMaterials = Assets::materials();
    _sceneMeshes = Assets::meshes();
}

void MainWindow::initializeScene()
{
    float aspect = float(width()) / height();
    graph::SceneObject* obj {};
    // auto obj = createCameraObject(aspect, "Main Camera");
    // obj->transform()->setLocalPosition({0,0,10});

    obj = createLightObject(Light::Type::Directional, "The Light");
    obj->transform()->setLocalEulerAngles({50,130,0});

    // createPrimitiveObject(*GLGraphics3::box(), "Box");

    // Load surfaces
    std::tuple<const char*, vec3f, vec3f, float> surfaces[] = {
        {"bezier/teacup", vec3f(0, 0, -2), vec3f::null(), 1.0},
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

    GLuint fs = *editor()->fragmentShader();
    auto setFragmentUniforms = [](GLRenderer& ctx)
    {
        ctx.defaultFragmentSubroutines();
    };

    auto p0 = new SurfacePipeline(0, fs, SurfacePipeline::Mode::Standard3D);
    p0->beforeDrawing = setFragmentUniforms;
    editor()->setPipeline(GLRenderer::Surface, p0);

    auto p1 = new SurfacePipeline(0, fs, SurfacePipeline::Mode::ContourCurves);
    p1->beforeDrawing = setFragmentUniforms;
    editor()->setPipeline("SurfCont"_ID, p1);

    registerInspectFunction(inspectSurface);

    _texRenderer = new SplRenderer();

#if SPL_BC_STATS
    {
        const uint32_t indexArray[]
        {
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
        };
        vec4f vertexArray[16];
        for (auto& v : vertexArray)
            v = {0,0,0,1};

        _debugPatch2D = new BezierPatches();
        auto idxs = _debugPatch2D->indexes();
        idxs->resize(16);
        idxs->setData(indexArray);
        auto points = _debugPatch2D->points();
        points->resize(16);
        points->setData(vertexArray);
        _debugObject = createSurfaceObject(*_debugPatch2D, "Debug Object");
    }
#endif
}

void MainWindow::render()
{
    if (_viewMode != ViewMode::Editor)
    {
        renderScene();
        // prevRenderScene();
        return;
    }

#if SPL_BC_STATS
    vec3f __debugLine {};
    auto n = spline::stats::g_BezierClippingData.size();
    if (n > 1 && _debugPatchIndex < n)
    {
        vec4f buffer[16];
        auto& data = spline::stats::g_BezierClippingData[_debugPatchIndex];
        auto& step = data.steps[_debugStep];
        auto patch2D = data.patch2D;
        if (_debugStep > 0)
        {
            auto [u0, v0] = step.min;
            auto [u1, v1] = step.max;
            spline::subpatch(patch2D.data(), u0, u1, v0, v1);
        }
        for (int i = 0; i < 16; i++)
        {
            auto& v = patch2D[i];
            buffer[i] = {v.x, 0, v.y, 1};
        }
        _debugPatch2D->points()->setData(buffer);
        __debugLine = {step.L.x, 0, step.L.y};
    }
#endif

    // glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0U);
    auto e = editor();
    auto& v = _state.renderViewport;
    e->setImageSize(v.w, v.h);
    glViewport(v.x, v.y, v.w, v.h);

    if (_sceneEnvironment)
    {
        auto& s = renderer()->fragmentShader()->samplers();
        int unit = s.sEnvironment.textureUnit;
        glActiveTexture(GL_TEXTURE0 + unit);
        glBindTexture(GL_TEXTURE_2D, _sceneEnvironment->handle());
        renderer()->setEnvironmentTextureUnit(unit);
    }

    e->render();

    if (e->showGround)
        e->drawXZPlane(10, 1);

    // Draw selected object
    if (auto obj = currentNode()->as<graph::SceneObject>())
    {
        const auto t = obj->transform();
        drawSelectedObject(*obj);
        if (_state.cursorMode == CursorMode::Select)
            editor()->drawTransform(t->position(), t->rotation());

        const SurfaceProxy* p = nullptr;
        for (graph::Component* c : obj->components())
            if (auto ptr = dynamic_cast<const SurfaceProxy*>(c))
            {
                p = ptr;
                break;
            }

        if (_state.cursorMode == CursorMode::NormalInspect
            && p != nullptr
            && _lastPickHit.object == obj)
        {
            SurfacePrimitive& s = p->mapper()->surface();
            vec3f n = s.normal(_lastPickHit);
            vec4f P = s.point(_lastPickHit);
            vec3f Q = vec3f(P.x / P.w, P.y / P.w, P.z / P.w);
            e->setVectorColor(Color{0x9c, 0x27, 0xb0}); // Purple A500
            e->setBasePoint(Q);
            e->drawVector(Q, n, e->pixelsLength(100.0));
        }
    }

#if SPL_BC_STATS
    // draw debug object
    drawSelectedObject(*_debugObject);
    auto t = _debugObject->transform();
    vec3f P = t->position();
    vec3f L = t->transformVector(__debugLine);
    e->setLineColor(Color::red);
    e->drawLine(P - 10.0f * L, P + 10.0f * L);
#endif

    // glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0U);
}

void MainWindow::convertScene()
{
    using Key = rt::Scene::Key;
    std::pmr::memory_resource* memoryResource = std::pmr::get_default_resource();
    auto objCount = _scene->actorCount();
    _rtScene = std::make_unique<rt::Scene>(objCount, memoryResource);

    auto& lights = _rtScene->lights;
    lights.clear();
    lights.reserve(_scene->lights().size());

    for (auto light : _scene->lights())
    {
        if (!light->isTurnedOn())
            continue;
        float angle;
        switch (light->type())
        {
        case Light::Type::Directional:
            angle = 0.0f;
            break;
        case Light::Type::Point:
            angle = 3.1415f;
            break;
        case Light::Type::Spot:
            angle = math::toRadians(light->spotAngle());
        }
        lights.push_back(rt::Light{
            .color = (vec3f) light->color,
            .strength = 1.0f,
            .position = light->position(),
            .range = light->range(),
            .direction = light->direction(),
            .angle = angle
        });
    }

    // Give some "pointer stability" to these arrays
    // All the required data must be previously allocated
    _rtScene->meshes.clear();
    _rtScene->meshes.reserve(objCount);
    _rtScene->surfaces.clear();
    _rtScene->surfaces.reserve(objCount);
    _rtScene->bvhs.clear();
    _rtScene->bvhs.reserve(objCount);

    auto& objects = _rtScene->objects;
    for (auto actor : _scene->actors())
    {
        if (!actor->isVisible())
            continue;

        auto mapper = actor->mapper();

        auto p = mapper->primitive();
        auto m = p->material();
        auto last = objects.size();
        objects.emplace_back();

        objects.get<Key::eLocal2WorldMatrix>(last) = p->localToWorldMatrix();
        objects.get<Key::eWorld2LocalMatrix>(last) = p->worldToLocalMatrix();

        objects.get<Key::eMaterial>(last) =
        {
            .diffuse = vec3f(m->diffuse),
            .specular = vec3f(m->specular),
            .transparency = vec3f(m->transparency),
            .metalness = m->metalness,
            .roughness = m->roughness,
            .refractiveIndex = m->ior,
        };

        if (auto p = dynamic_cast<TriangleMeshMapper*>(mapper))
        {
            auto& data = p->mesh()->data();
            auto& mesh = _rtScene->meshes.emplace_back();
            auto indexCount = 3 * data.triangleCount;
            mesh.vertices = _rtScene->createBuffer<vec3f>(data.vertexCount);
            mesh.normals = _rtScene->createBuffer<vec3f>(data.vertexCount);
            mesh.indices = _rtScene->createBuffer<uint32_t>(indexCount);
            std::copy_n(data.vertices, data.vertexCount, mesh.vertices);
            std::copy_n(data.vertexNormals, data.vertexCount, mesh.normals);
            for (int i = 0, j = 0; i < data.triangleCount; i++, j += 3)
            {
                mesh.indices[j  ] = data.triangles[i].v[0];
                mesh.indices[j+1] = data.triangles[i].v[1];
                mesh.indices[j+2] = data.triangles[i].v[2];
            }

            objects.get<Key::ePrimitive>(last) = &mesh;
        }
        else if (auto s = dynamic_cast<SurfaceMapper*>(mapper))
        {
            auto patches = s->surface().patches();
            auto points = patches->points();
            auto indices = patches->indexes();
            auto pointsPtr = points->map(GL_READ_ONLY);
            auto indicesPtr = indices->map(GL_READ_ONLY);
            auto& surface = _rtScene->surfaces.emplace_back();
            surface.vertices = _rtScene->createBuffer<vec4f>(points->size());
            surface.indices = _rtScene->createBuffer<uint32_t>(indices->size());
            surface.indexCount = indices->size();
            std::copy_n(pointsPtr.get(), points->size(), surface.vertices);
            std::copy_n(indicesPtr.get(), indices->size(), surface.indices);

            auto& bvh = _rtScene->bvhs.emplace_back(memoryResource);
            surface.buildBVH(bvh);

            objects.get<Key::ePrimitive>(last) = &surface;
        }
    }

    _rtScene->buildBVH();

    rt::PerfectHashTable table;
    for (auto& b : _rtScene->bvhs)
        table.build(b.keys());

    table.build(_rtScene->topLevelBVH.keys());
}

void MainWindow::renderScene()
{
    if (_viewMode != ViewMode::Renderer)
        return;

    if (_image != nullptr)
    {
        // draw
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, *_image);
        _texRenderer->drawTexture2D(0);
        return;
    }

    auto camera = graph::CameraProxy::current();
    if (!camera)
        camera = editor()->camera();

#if SPL_BC_STATS
    spline::stats::g_BezierClippingData.clear();
    spline::stats::g_BezierClippingEnable = false;
#endif

    convertScene();

    // Set Camera
    _rtCamera =
    {
        .transform =
        {
            .rotation = camera->rotation(),
            .position = camera->position(),
        },
        .aspectRatio = camera->aspectRatio(),
        .fieldOfView = math::toRadians(camera->viewAngle()),
        .height = camera->height(),
        .projection = camera->projectionType() == Camera::Parallel
            ? rt::Camera::Parallel : rt::Camera::Perspective
    };

    // Create task
    // if (!_renderTask.valid()) {}
    auto& v = _state.renderViewport;
    _image = new gl::Texture(gl::Format::sRGB, v.w, v.h);

    _frame = new rt::Frame(v.w, v.h);
    _rayTracer = new rt::CPURayTracer({
        .backgroundColor = vec3f(backgroundColor),
        .flipYAxis = true,
        .threads = 1,
    });
    _rayTracer->render(_frame, &_rtCamera, _rtScene.get());

    glTextureSubImage2D(*_image, 0, 0, 0, v.w, v.h, GL_RGBA, GL_UNSIGNED_BYTE,
        _frame->data());

#if SPL_BC_STATS
    spline::stats::g_BezierClippingEnable = true;
#endif
}

void MainWindow::prevRenderScene()
{
    if (_viewMode != ViewMode::Renderer)
        return;

    auto camera = graph::CameraProxy::current();
    if (!camera)
        camera = editor()->camera();

    if (_prevImage == nullptr)
    {
#if SPL_BC_STATS
        spline::stats::g_BezierClippingData.clear();
        spline::stats::g_BezierClippingEnable = false;
#endif

        // Create task
        _prevImage = new GLImage(width(), height());
        if (_prevRayTracer == nullptr)
            _prevRayTracer = new RayTracer(*scene(), *camera);
        else
            _prevRayTracer->setCamera(*camera);
        // _prevRayTracer->setProgressCallback(progressCallback);
        _prevRayTracer->setMaxRecursionLevel(6);
        _prevRayTracer->renderImage(*_prevImage);

#if SPL_BC_STATS
        spline::stats::g_BezierClippingEnable = true;
#endif
    }
    _prevImage->draw(0, 0);
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
    auto dockId = ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(),
        ImGuiDockNodeFlags_PassthruCentralNode
        | ImGuiDockNodeFlags_NoDockingInCentralNode);
    auto node = ImGui::DockBuilderGetCentralNode(dockId);

    if (_state.renderOnCentralNode)
        _state.renderViewport =
        {
            .x = (int) node->Pos.x,
            .y = framebufferHeight - (int(node->Pos.y) + int(node->Size.y)),
            .w = (int) node->Size.x,
            .h = (int) node->Size.y
        };
    else
        _state.renderViewport =
        {
            .x = 0,
            .y = 0,
            .w = framebufferWidth,
            .h = framebufferHeight
        };

    mainMenu();
    assetWindow();
    hierarchyWindow();
    inspectorWindow();
    editorView();
    controlWindow();
    ImGui::ShowMetricsWindow();
}

struct BlitFBCallbackData
{
    GLuint framebuffer;
};

static
void BlitFramebufferImGuiWindow(const ImDrawList* parentList,
    const ImDrawCmd* cmd)
{
    // Not tested
    // Save
    GLuint lastReadFramebuffer;
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, (GLint*) &lastReadFramebuffer);

    // Draw framebuffer
    auto data = (const BlitFBCallbackData*) cmd->UserCallbackData;
    // Not tested with multiple viewports branch (it may not work)
    // For multiple viewports ClipRect must be subtracted by DisplayPos
    auto x0 = cmd->ClipRect.x;
    auto y0 = cmd->ClipRect.y;
    auto x1 = cmd->ClipRect.z;
    auto y1 = cmd->ClipRect.w;
    // glScissor(x0, y0, x1, y1);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, data->framebuffer);
    glBlitFramebuffer(
        0, 0, x1 - x0, y1 - y0,
        x0, y0, x1, y1,
        GL_COLOR_BUFFER_BIT,
        GL_NEAREST
    );

    // Restore
    glBindFramebuffer(GL_READ_FRAMEBUFFER, lastReadFramebuffer);
}

void MainWindow::cameraPreview()
{
    if (ImGui::Begin("Camera", nullptr, 0))
    {
        auto& fbo = _state.cameraFramebuffer;
        auto viewport = ImGui::GetWindowViewport();
        fbo->resize(
            uint32_t(viewport->WorkSize.x),
            uint32_t(viewport->WorkSize.y)
        );

        static BlitFBCallbackData data { .framebuffer = fbo->handle() };
        ImGui::GetWindowDrawList()
            ->AddCallback(BlitFramebufferImGuiWindow, &data);
    }
    ImGui::End(); // Always call end
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
            if (ImGui::MenuItem(file.filename().string().c_str()))
                readScene(file);
        ImGui::EndMenu();
    }
}

inline
void MainWindow::viewMenu()
{
    if (ImGui::BeginMenu("View"))
    {
        (void)ImGui::MenuItem("Render on central node", nullptr,
            &_state.renderOnCentralNode);
        ImGui::EndMenu();
    }
}

inline
void MainWindow::helpMenu()
{
    if (ImGui::BeginMenu("Help"))
    {
        if (ImGui::MenuItem("Keybindings"))
            (void)0;
        ImGui::EndMenu();
    }
}

inline
void MainWindow::mainMenu()
{
    if (ImGui::BeginMainMenuBar())
    {
        fileMenu();
        viewMenu();
        helpMenu();
        ImGui::EndMainMenuBar();
    }
}

static
const char* componentTypeString(GLint type)
{
    const char* s = "<unknown>";
    switch (type)
    {
    case GL_FLOAT:                  s = "FLOAT";
    case GL_INT:                    s = "INT";
    case GL_UNSIGNED_INT:           s = "UNSIGNED INT";
    case GL_SIGNED_NORMALIZED:      s = "SIGNED NORMALIZED";
    case GL_UNSIGNED_NORMALIZED:    s = "UNSIGNED NORMALIZED";
    }
    return s;
}

static
const char* framebufferStatusString(GLenum status)
{
    const char* s = "Incomplete";
    switch (status)
    {
        case GL_FRAMEBUFFER_COMPLETE:
            s = "Complete"; break;
        case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
            s = "Incomplete attachment"; break;
        case GL_FRAMEBUFFER_UNDEFINED:
            s = "Undefined"; break;
        case GL_FRAMEBUFFER_UNSUPPORTED:
            s = "Unsupported"; break;
        case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
            s = "Incomplete Draw buffer"; break;
        case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
            s = "Incomplete Read buffer"; break;
        case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
            s = "Incomplete Multisample"; break;
        case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
            s = "Incomplete layer targets"; break;
        case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
            s = "Incomplete (missing attachment)"; break;
    };
    return s;
}

void MainWindow::assetWindow()
{
    if (!_showAssets) return;

    ImGui::Begin("Assets");

    auto flags = ImGuiTreeNodeFlags_DefaultOpen;

    if (ImGui::CollapsingHeader("Materials", flags))
    {
        for (auto& [name, m] : _sceneMaterials)
        {
            auto d = "Diffuse##"+name;
            auto s = "Specular##"+name;
            bool selected = false;

            ImGui::ColorButton(d.c_str(), *(ImVec4*)&m->diffuse);
            ImGui::SameLine();
            ImGui::ColorButton(s.c_str(), *(ImVec4*)&m->specular);
            ImGui::SameLine();

            ImGui::Selectable(name.c_str(), &selected);
        }
    }

    if (ImGui::CollapsingHeader("Meshes", flags))
    {
        for (auto& [name, m] : _sceneMeshes)
        {
            bool selected = false;
            ImGui::Selectable(name.c_str(), &selected);
        }
    }

    if (ImGui::CollapsingHeader("PBR Materials", flags))
    {
        for (auto& [name, m] : _scenePBRMaterials)
        {
            auto d = "Base Color##"+name;
            bool selected = false;

            ImGui::ColorButton(d.c_str(), *(ImVec4*)&m->baseColor);
            ImGui::SameLine();

            ImGui::Selectable(name.c_str(), &selected);
        }
    }

    if (ImGui::CollapsingHeader("Textures", flags))
    {
        static bool view {true};
        static gl::Texture* selected {nullptr};

        ImGui::Checkbox("View Texture", &view);
        ImGui::Spacing();

        for (auto& [name, t] : _sceneTextures)
        {
            if (ImGui::Selectable(name.c_str(), t.get() == selected))
            {
                selected = t.get() == selected ? nullptr : t;
            }
        }

        if (selected && view)
        {
            if (ImGui::Begin("Texture", &view))
            {
                auto id = reinterpret_cast<ImTextureID>((uintptr_t) selected->handle());
                auto max = ImGui::GetContentRegionMax();
                auto min = ImGui::GetWindowContentRegionMin();
                ImGui::Image(id, {max.x - min.x, max.y - min.y});
            }
            ImGui::End();
        }
    }

    ImGui::End();
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
    int mode = _state.cursorMode;
    if (ImGui::Combo("Cursor Mode", &mode, items, std::size(items)))
        _state.cursorMode = modes[mode];

    ImGui::Separator();

    ImGui::Text("Color Encoding: %s",
        _data.windowColorEncoding == GL_SRGB ? "sRGB" : "Linear");
    ImGui::Text("Component Type: %s",
        componentTypeString(_data.windowComponentType));
    ImGui::Text("Red Channel:   %d bits", _data.windowRedBits);
    ImGui::Text("Green Channel: %d bits", _data.windowGreenBits);
    ImGui::Text("Blue Channel:  %d bits", _data.windowBlueBits);

    ImGui::Text("Alternative framebuffer: %s",
        framebufferStatusString(_state.cameraFramebuffer->status()));

    ImGui::Text("Samples: %d", _data.windowSamples);
    ImGui::Text("Sample Buffers: %d", _data.windowSampleBuffers);

    ImGui::Separator();

    static float depthValue = 0.99999f;
    ImGui::DragFloat("Environment Depth", &depthValue, 0.0001);
    auto p = editor()->__environmentProg();
    p->disuse();
    p->use();
    p->setDepth(depthValue);

#if SPL_BC_STATS
    ImGui::Separator();
    auto t = _debugObject->transform();
    float scale = std::log10(t->localScale().x);
    float slidermin = -3;
    float slidermax = 20;
    if (ImGui::SliderScalar("Debug Zoom", ImGuiDataType_Float, &scale,
        &slidermin, &slidermax))
    {
        t->setLocalScale(std::pow(10, scale));
    }

    ImGui::Separator();
    char label[128];
    const auto& patches = spline::stats::g_BezierClippingData;
    ImGui::Text("Patch intersection test data (%d)", patches.size());
    if (ImGui::BeginListBox("##PatchList", {-FLT_MIN, -FLT_MIN}))
    {
        for (int i = 0; i < patches.size(); i++)
        {
            const auto& p = patches[i];
            const bool selected = i == _debugPatchIndex;
            const uint32_t flags = ImGuiTreeNodeFlags_DefaultOpen
                | ImGuiTreeNodeFlags_OpenOnArrow
                | ImGuiTreeNodeFlags_OpenOnDoubleClick
                | ImGuiTreeNodeFlags_SpanAvailWidth
                | (selected ? ImGuiTreeNodeFlags_Selected : 0);

            snprintf(label, sizeof (label),
                "%d (max depth = %d)", i, p.maxStackDepth);

            auto open = ImGui::TreeNodeEx(label, flags);
            if (ImGui::IsItemClicked())
                _debugPatchIndex = i, _debugStep = 0;
            if (open)
            {
                for (auto& hit : p.hits)
                {
                    snprintf(label, sizeof (label), "(%.4f, %.4f) d=%f",
                        hit.coord.x, hit.coord.y, hit.distance);
                    ImGui::Selectable(label);
                    // ImGui::SetItemAllowOverlap();
                }

                if (ImGui::TreeNode("Steps"))
                {
                    for (int j = 0; j < p.steps.size(); j++)
                    {
                        auto& s = p.steps[j];
                        float delta = std::max(s.max.x - s.min.x,
                            s.max.y - s.min.y);
                        float reduction = s.lower > s.upper
                            ? NAN
                            : 100 * ((s.upper - s.lower) - 1);
                        snprintf(label, sizeof (label),
                            "u = [%f, %f]\n"
                            "v = [%f, %f]\n"
                            "Δ = %f\n"
                            "clip = [%f, %f]\n"
                            "%2.2f%% reduction\n"
                            "cutside = '%c'",
                            s.min.x, s.max.x,
                            s.min.y, s.max.y,
                            delta,
                            s.lower, s.upper,
                            reduction,
                            s.cutside
                        );
                        const bool selected2 = selected && _debugStep == j;
                        if (ImGui::Selectable(label, selected2))
                            _debugPatchIndex = i, _debugStep = j;
                    }
                    ImGui::TreePop();
                }

                ImGui::TreePop();
            }

            if (selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndListBox();
    }
#endif

    ImGui::End();
}

void MainWindow::inspectSurface(MainWindow& window, SurfaceProxy& s)
{
    ImGui::inputText("Surface", s.sceneObject()->name());
    ImGui::Separator();
    window.inspectMaterial(s.mapper()->surface());
}

bool MainWindow::onKeyPress(int key, int p2)
{
    if (_viewMode == ViewMode::Editor)
    {
        switch (key)
        {
        case GLFW_KEY_F1:
            _state.cursorMode = CursorMode::Select; break;
        case GLFW_KEY_F2:
            _state.cursorMode = CursorMode::PrimitiveInspect; break;
        case GLFW_KEY_F3:
            _state.cursorMode = CursorMode::NormalInspect; break;
        }
    }

    return SceneWindow::onKeyPress(key, p2);
}

bool MainWindow::onMouseLeftPress(int x, int y)
{
#if SPL_BC_STATS
    spline::stats::g_BezierClippingData.clear();
#endif

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

bool MainWindow::onResize(int width, int height)
{
    // _state.framebuffer->resize(width, height);
    return true;
}

graph::SceneObject *MainWindow::pickObject(int x, int y) const
{
    auto& v = _state.renderViewport;
    auto ray = makeRay(x - v.x, y - v.y);
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

    reader.setInput(scenefile.string());
    try
    {
        reader.execute();
        if (reader.scene() == nullptr)
            throw std::runtime_error("Scene is null");
        setScene(*reader.scene());

        // auto& materials = Assets::materials();
        // for (auto& [name, m] : reader.materials)
        //     materials[name] = m;

        _sceneEnvironment = reader.environment;
        _sceneMaterials = std::move(reader.materials);
        _scenePBRMaterials = std::move(reader.pbrMaterials);
        _sceneSurfaces = std::move(reader.surfaces);
        _sceneTextures = std::move(reader.textures);
    }
    catch (const std::exception& e)
    {
        std::cerr << std::format("Failed to load scene '{}'. Reason:\n{}\n",
            scenefile.filename().string().c_str(), e.what());
    }
}

graph::SceneObject*
MainWindow::createSurfaceObject(BezierPatches &p, const char *name)
{
    auto object = graph::SceneObject::New(*_scene);

    object->setName("%s", name);
    object->addComponent(new SurfaceProxy(new SurfacePrimitive(&p)));

    return object;
}

} // namespace cg
