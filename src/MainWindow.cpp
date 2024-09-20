#include "MainWindow.h"

#include <graphics/Application.h>
#include <graphics/Assets.h>
#include <graph/SceneObject.h>
#include <graph/SceneNode.h>
#include <format>
#include <map>
#include <ranges>
#include "Framebuffer.h"
#include "Log.h"
#include "SceneReaderExt.h"
// #include "Surface.h"
#include "SplineMat.h"
#include "reader/SceneReader.h"
#include "rt/ScopeClock.h"

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

    _sceneEnvironment = gl::Texture::from("assets/textures/je_gray_park_4k.hdr");

    Reference light = new Light();
    light->color = Color(0.4f, 0.4f, 0.4f);
    obj = createObject("The Light", graph::LightProxy::New(*light));
    obj->transform()->setLocalEulerAngles({50,130,0});

    obj = createPrimitiveObject(*GLGraphics3::box(), "Box");
    obj->transform()->setLocalScale({10, 0.5f, 10});
    obj->transform()->setPosition({0, -1, 0});

    // Load surfaces
    std::tuple<const char*, vec3f, vec3f, float> surfaces[] = {
        {"bezier/teacup", vec3f(0, 0, -2), vec3f::null(), 1.0},
        {"bezier/teapot", vec3f(-2.5, 0, 0), vec3f(-90, 0, 0), 0.5},
        {"bezier/teaspoon", vec3f(1.5, 0, 0), vec3f::null(), 1.0}
    };

    for (auto& [s, t, r, sc] : surfaces)
    {
        auto asset = Application::assetFilePath(s);
        auto p = GLBezierSurface::load(asset.c_str());
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

    editor()->camera()->setPosition({1, 3.3, 3.4});
    editor()->camera()->setEulerAngles({-36, 28, 9.5e-6});

    _texRenderer = new SplRenderer();

// #if SPL_BC_STATS
    {
        const uint32_t indexArray[]
        {
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
        };
        vec4f vertexArray[16];
        for (auto& v : vertexArray)
            v = {0,0,0,1};

        _debugPatch2D = new GLBezierSurface();
        auto idxs = _debugPatch2D->indices();
        idxs->resize(16);
        idxs->setData(indexArray);
        auto points = _debugPatch2D->points();
        points->resize(16);
        points->setData(vertexArray);
        _debugObject = createSurfaceObject(*_debugPatch2D, "Debug Object");
    }
// #endif
}

void MainWindow::render()
{
    // glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0U);
    auto e = editor();
    auto& v = _state.renderViewport;
    e->setImageSize(v.w, v.h);
    glViewport(v.x, v.y, v.w, v.h);

    if (_image != nullptr)
    {
        auto dt = glIsEnabled(GL_DEPTH_TEST);
        glDisable(GL_DEPTH_TEST);
        // draw
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, *_image);
        _texRenderer->drawTexture2D(0);
        if (dt)
            glEnable(GL_DEPTH_TEST);
        return;
    }

// #if SPL_BC_STATS
    // vec3f __debugLine {};
    // auto n = spline::stats::g_BezierClippingData.size();
    // if (n > 1 && _debugPatchIndex < n)
    // {
    //     vec4f buffer[16];
    //     auto& data = spline::stats::g_BezierClippingData[_debugPatchIndex];
    //     auto& step = data.steps[_debugStep];
    //     auto patch2D = data.patch2D;
    //     if (_debugStep > 0)
    //     {
    //         auto [u0, v0] = step.min;
    //         auto [u1, v1] = step.max;
    //         spline::subpatch(patch2D.data(), u0, u1, v0, v1);
    //     }
    //     for (int i = 0; i < 16; i++)
    //     {
    //         auto& v = patch2D[i];
    //         buffer[i] = {v.x, 0, v.y, 1};
    //     }
    //     _debugPatch2D->points()->setData(buffer);
    //     __debugLine = {step.L.x, 0, step.L.y};
    // }
// #endif

    if (_sceneEnvironment)
    {
        auto& s = renderer()->fragmentShader()->samplers();
        int unit = s.sEnvironment.textureUnit;
        glActiveTexture(GL_TEXTURE0 + unit);
        glBindTexture(GL_TEXTURE_2D, _sceneEnvironment->handle());
        renderer()->setEnvironmentTextureUnit(unit);
    }
    else
        renderer()->setEnvironmentTextureUnit(-1);

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

    if (auto obj = (const graph::SceneObject*) _lastPickHit.object)
    {
        const SurfaceProxy* ptr = nullptr;
        for (const graph::Component* c : obj->components())
        {
            if (ptr = dynamic_cast<const SurfaceProxy*>(c))
            {
                auto s = ptr->mapper()->surface().patches();
                auto points = s->points()->map(GL_READ_ONLY);
                auto indices = s->indices()->map(GL_READ_ONLY);
                int base = 16 * _lastPickHit.triangleIndex;
                vec4 buffer[16];
                for (int i = 0; i < 16; i++)
                {
                    buffer[i] = points[indices[base + i]];
                }
                auto [u0, u1] = _debugClip[0];
                auto [v0, v1] = _debugClip[1];
                _matClip
                    ? spline::mat::subpatch(buffer, u0, u1, v0, v1)
                    : spline::subpatch(buffer, u0, u1, v0, v1);
                _debugPatch2D->points()->setData(buffer);
                break;
            }
        }
    }

    // draw debug object
    // drawSelectedObject(*_debugObject);

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
    using PrType = rt::PrimitiveType;
    std::pmr::memory_resource* memoryResource = rt::ManagedResource::instance();
    auto objCount = _scene->actorCount();
    _rtScene = std::make_unique<rt::Scene>(objCount, memoryResource);
    _rtScene->backgroundColor = vec3(_scene->backgroundColor);

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

        objects.get<Key::eLocal2WorldMatrix>(last) = mat4(p->localToWorldMatrix());
        objects.get<Key::eWorld2LocalMatrix>(last) = mat4(p->worldToLocalMatrix());

        objects.get<Key::eMaterial>(last) =
        {
            .diffuse = vec3(m->diffuse),
            .specular = vec3(m->specular),
            .transparency = vec3(m->transparency),
            .metalness = m->metalness,
            .roughness = m->roughness,
            .refractiveIndex = m->ior,
        };

        if (auto p = dynamic_cast<TriangleMeshMapper*>(mapper))
        {
            auto& data = p->mesh()->data();
            auto& mesh = _rtScene->meshes.emplace_back();
            mesh.indexCount = 3 * data.triangleCount;
            mesh.vertices = _rtScene->createBuffer<vec3>(data.vertexCount);
            mesh.normals = _rtScene->createBuffer<vec3>(data.vertexCount);
            mesh.indices = _rtScene->createBuffer<uint32_t>(mesh.indexCount);
            std::copy_n(data.vertices, data.vertexCount, mesh.vertices);
            std::copy_n(data.vertexNormals, data.vertexCount, mesh.normals);
            for (int i = 0, j = 0; i < data.triangleCount; i++, j += 3)
            {
                mesh.indices[j  ] = data.triangles[i].v[0];
                mesh.indices[j+1] = data.triangles[i].v[1];
                mesh.indices[j+2] = data.triangles[i].v[2];
            }

            auto& bvh = _rtScene->bvhs.emplace_back(memoryResource);
            mesh.buildBVH(bvh);

            objects.get<Key::ePrimitive>(last) = &mesh;
            objects.get<Key::ePrimitiveType>(last) = PrType::eMesh;
        }
        else if (auto s = dynamic_cast<SurfaceMapper*>(mapper))
        {
            auto patches = s->surface().patches();
            auto points = patches->points();
            auto indices = patches->indices();
            auto pointsPtr = points->map(GL_READ_ONLY);
            auto indicesPtr = indices->map(GL_READ_ONLY);
            auto& surface = _rtScene->surfaces.emplace_back();
            surface.vertices = _rtScene->createBuffer<vec4>(points->size());
            surface.indices = _rtScene->createBuffer<uint32_t>(indices->size());
            surface.indexCount = indices->size();
            std::copy_n(pointsPtr.get(), points->size(), surface.vertices);
            std::copy_n(indicesPtr.get(), indices->size(), surface.indices);

            auto& bvh = _rtScene->bvhs.emplace_back(memoryResource);
            surface.buildBVH(bvh);

            objects.get<Key::ePrimitive>(last) = &surface;
            objects.get<Key::ePrimitiveType>(last) = PrType::eBezierSurface;
        }
    }

    _rtScene->buildBVH();

    for (auto& b : _rtScene->bvhs)
        b.buildHashTable();

    _rtScene->topLevelBVH.buildHashTable();
}

void MainWindow::renderScene()
{
    // if (_viewMode != ViewMode::Renderer)
    //     return;

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
    auto& v = _state.renderViewport;
    _image = new gl::Texture(gl::Format::sRGB, v.w, v.h);

    constexpr int warpSize = 32;
    _frame = new rt::Frame(v.w, v.h, warpSize, rt::ManagedResource::instance());

    GLint rowLength;
    glGetIntegerv(GL_UNPACK_ROW_LENGTH, &rowLength);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, _frame->stride());

    switch (_renderMethod)
    {
    case RenderMethod::eCUDA:
    {
        rt::ScopeClock _c {[](std::chrono::microseconds duration)
        {
            log::info("CUDA ray tracing in {} ms", duration.count() / 1000.0f);
        }};

        auto heatMap = rt::makeManaged<rt::Frame>(v.w, v.h, warpSize, rt::ManagedResource::instance());
        CUDA_CHECK(cudaMemsetAsync(heatMap->data(), 0, heatMap->size_bytes()));

        // Launch render kernel
        _rayTracer = new rt::RayTracer({
            .device = 0,
            .heatMap = heatMap.get(),
        });
        _rayTracer->render(_frame, &_rtCamera, _rtScene.get());

        CUDA_CHECK(cudaDeviceSynchronize());
        _workbench2D["CUDA ray traced image"] = _image;

        // Get heat map
        Ref<gl::Texture> tex = new gl::Texture(gl::Format::sRGB, v.w, v.h);
        _workbench2D["Bezier Clipping Heat Map"] = tex;

        // Convert counters to colors
        for (auto i = 0U; i < heatMap->size(); i++)
        {
            auto& e = heatMap->data()[i];
            e = packColor(Color(heatPallete(e)));
        }

        glTextureSubImage2D(*tex, 0, 0, 0, v.w, v.h, GL_RGBA, GL_UNSIGNED_BYTE,
            heatMap->data());
    } break;
    case RenderMethod::eCPU:
    {
        _cpuRayTracer = new rt::CPURayTracer(_cpuRTOptions);
        _cpuRayTracer->render(_frame, &_rtCamera, _rtScene.get());

        int m = _cpuRayTracer->options().tileSize;
        m *= m;
        auto status = _cpuRayTracer->status();
        auto total = status->totalWork;
        while (status->running.test())
        {
            using namespace std::chrono_literals;
            auto done = status->workDone.load();
            float p = (100.0f * done) / total;
            int a = m * done;
            int b = m * total;
            printf("\rRay traced %d of %d pixels (%.2f%%)", a, b, p);
            std::this_thread::sleep_for(67ms);
        }
        _workbench2D["Ray traced image"] = _image;
    } break;
    default:
        break;
    }

    glTextureSubImage2D(*_image, 0, 0, 0, v.w, v.h, GL_RGBA, GL_UNSIGNED_BYTE,
        _frame->data());

    glPixelStorei(GL_UNPACK_ROW_LENGTH, rowLength);

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

bool MainWindow::onKeyPress(int key, int p2)
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

// void MainWindow::readScene(std::filesystem::path scenefile)
// {
//     util::SceneReaderExt reader {};

//     reader.setInput(scenefile.string());
//     try
//     {
//         reader.execute();
//         if (reader.scene() == nullptr)
//             throw std::runtime_error("Scene is null");
//         setScene(*reader.scene());

//         // auto& materials = Assets::materials();
//         // for (auto& [name, m] : reader.materials)
//         //     materials[name] = m;

//         _sceneEnvironment = reader.environment;
//         _sceneMaterials = std::move(reader.materials);
//         _scenePBRMaterials = std::move(reader.pbrMaterials);
//         _sceneSurfaces = std::move(reader.surfaces);
//         _sceneTextures = std::move(reader.textures);
//     }
//     catch (const std::exception& e)
//     {
//         std::cerr << std::format("Failed to load scene '{}'. Reason:\n{}\n",
//             scenefile.filename().string().c_str(), e.what());
//     }
// }

void MainWindow::readScene(std::filesystem::path scenefile)
{
    SceneReader reader {};

    auto file = scenefile.string();
    std::fstream in (file, std::ios::in);
    if (!in)
    {
        log::error("File not found: {}", file);
        return;
    }

    try
    {
        reader.parse(scenefile.filename().string(), in, scenefile.parent_path());
        if (reader.scene() == nullptr)
            throw std::runtime_error("Scene is null");
        setScene(*reader.scene());

        _sceneEnvironment = reader.environment;
    }
    catch (const std::exception& e)
    {
        std::cerr << std::format("Failed to load scene '{}'. Reason:\n{}\n",
            scenefile.filename().string().c_str(), e.what());
    }
}

graph::SceneObject*
MainWindow::createSurfaceObject(GLBezierSurface &p, const char *name)
{
    auto object = graph::SceneObject::New(*_scene);

    object->setName("%s", name);
    object->addComponent(new SurfaceProxy(new SurfacePrimitive(&p)));

    return object;
}

} // namespace cg
