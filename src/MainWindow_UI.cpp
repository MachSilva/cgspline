#include "MainWindow.h"

#include <fstream>
#include <imgui_internal.h>
#include "Log.h"
#include "stb_image_write.h"

namespace cg
{

void MainWindow::gui()
{
    if (auto fn = _state.backgroundTaskStatusWindow)
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(200ms);
        (this->*fn)();
        return;
    }

    auto dockId = ImGui::DockSpaceOverViewport(0U,
        ImGui::GetMainViewport(),
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

    // mainMenu();
    assetWindow();
    hierarchyWindow();
    inspectorWindow();
    editorView();
    controlWindow();
    workbenchWindow();
    // ImGui::ShowMetricsWindow();
}

void MainWindow::cpuRayTracerStatusWindow()
{
    using namespace std::chrono;

    auto center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f,0.5f));
    ImGui::Begin(c_CPURayTracerStatusTitle, nullptr, c_StatusWindowFlags);

    auto options = &_cpuRayTracer->options;
    auto s = options->nSamples;
    auto m = options->tileSize;
    m *= m;
    auto status = _cpuRayTracer->status();
    auto done = status->workDone.load();
    auto total = status->totalWork;
    int a = m * done;
    int b = m * total;
    ImGui::ProgressBar(float(done) / total, ImVec2(0,0));
    ImGui::SameLine();

    auto elapsed = duration_cast<milliseconds>
        (steady_clock::now() - status->started).count();
    auto e_m = elapsed / 60000;
    auto e_s = (elapsed % 60000) / 1000;
    auto e_ms = elapsed % 1000;
    ImGui::Text("%02lld:%02lld.%03lld", e_m, e_s, e_ms);

    ImGui::Text("Ray traced %d of %d pixels", a, b);
    ImGui::Text("Traced %d of %d rays", s*a, s*b);

    if (ImGui::Button("Cancel"))
    {
        _cpuRayTracer->stop();
    }
    // when finished close window and store image
    if (_cpuRayTracer->running() == 0)
    {
        whenCPURayTracerEnds();
        _state.backgroundTaskStatusWindow = nullptr;
    }
    ImGui::End();
}

void MainWindow::cudaRayTracerStatusWindow()
{
    using namespace std::chrono;

    auto center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f,0.5f));
    ImGui::Begin(c_CUDARayTracerStatusTitle, nullptr, c_StatusWindowFlags);

    ImGui::ProgressBar(-0.1, ImVec2(0,0));
    ImGui::SameLine();

    auto elapsed = duration_cast<milliseconds>
        (steady_clock::now() - _lastRenderStarted).count();
    auto e_m = elapsed / 60000;
    auto e_s = (elapsed % 60000) / 1000;
    auto e_ms = elapsed % 1000;
    ImGui::Text("%02lld:%02lld.%03lld", e_m, e_s, e_ms);

    ImGui::BeginDisabled();
    ImGui::Button("Cancel");
    ImGui::EndDisabled();
    // when finished close popup and store image
    if (auto e = cudaEventQuery(_rayTracer->finished); e == cudaSuccess)
    {
        whenCUDARayTracerEnds();
        _state.backgroundTaskStatusWindow = nullptr;
    }
    else if (e != cudaErrorNotReady)
    {
        log::error("CUDA rendering failed ({}): {}",
            cudaGetErrorName(e),
            cudaGetErrorString(e)
        );
        _state.backgroundTaskStatusWindow = nullptr;
    }
    ImGui::End();
}

static std::string _ImageFilename(std::string_view prefix, std::string_view ext)
{
    using std::chrono::system_clock;
    auto now = system_clock::to_time_t(system_clock::now());
    auto lt = localtime(&now);
    return std::format("{}_{}{:02}{:02}_{:02}{:02}{:02}.{}", prefix,
        lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
        lt->tm_hour, lt->tm_min, lt->tm_sec,
        ext);
}

void MainWindow::whenCPURayTracerEnds()
{
    _workbench2D["CPU ray traced image"] = _image;

    auto s = _cpuRayTracer->status();
    std::chrono::duration<double,std::milli>
        duration = s->finished - s->started;
    log::info("Ray tracing completed in {} ms", duration.count());

    auto w = _image->width();
    auto h = _image->height();

    GLint rowLength;
    glGetIntegerv(GL_UNPACK_ROW_LENGTH, &rowLength);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, _frame->stride());
    glTextureSubImage2D(*_image, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE,
        _frame->data());
    glPixelStorei(GL_UNPACK_ROW_LENGTH, rowLength);

    if (!_saveImages)
        return;

    auto stem = _sceneRefs.filepath.stem().string() + "_rt_cpu";
    auto f = _ImageFilename(stem, "png");
    int rc = stbi_write_png(f.c_str(), w, h, 4, _frame->data(), _frame->stride() * sizeof (rt::Frame::Pixel));
    if (rc == 0)
        log::error("Failed to write image \"{}\": {}", f, stbi_failure_reason());
    else
        log::info("Image written to \"{}\"", f);
}

void MainWindow::whenCUDARayTracerEnds()
{
    float elapsed;
    CUDA_CHECK(cudaEventElapsedTime(&elapsed, _rayTracer->started, _rayTracer->finished));
    _workbench2D["CUDA ray traced image"] = _image;

    _lastRenderInfo.clear();
    std::format_to(std::back_inserter(_lastRenderInfo),
        "CUDA ray tracing in {:.2f} ms", elapsed
    );

    log::info("{}", _lastRenderInfo);

    auto w = _image->width();
    auto h = _image->height();

    GLint rowLength;
    glGetIntegerv(GL_UNPACK_ROW_LENGTH, &rowLength);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, _frame->stride());
    glTextureSubImage2D(*_image, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE,
        _frame->data());

    auto heatMap = _rayTracer->options.heatMap;
    if (!heatMap)
        return;

    // Get heat map
    Ref<gl::Texture> tex = new gl::Texture(gl::Format::sRGB, w, h);
    _workbench2D["Bezier Clipping Heat Map"] = tex;

#ifdef RT_ENABLE_COUNTERS
    {
        std::fstream io ("clipping_stats.raw", std::ios::binary | std::ios::trunc | std::ios::out);
        int s = heatMap->stride();
        io << ".raw";
        io.write((char*)&w, 4);
        io.write((char*)&h, 4);
        io.write((char*)&s, 4);
        io.write((char*)heatMap->data(), heatMap->size_bytes());
        io.close();
    }

    auto counters = _rayTracer->counters;
    std::cout << "Rays traced: " << (counters->lightrays + counters->shadowrays)
        << "\nDepth histogram: [";
    for (int i = 0; i < 32; i++)
    {
        std::cout << counters->depthHistogram[i] << ',';
    }
    std::cout << "]\nIteration histogram: [";
    for (int i = 0; i < 64; i++)
    {
        std::cout << counters->iterationHistogram[i] << ',';
    }
    std::cout << "]\n";
#endif

    // Convert counters to colors
    for (auto i = 0U; i < heatMap->size(); i++)
    {
        auto& e = heatMap->data()[i];
        e = packColor(Color(heatPallete(e)));
    }

    glTextureSubImage2D(*tex, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE,
        heatMap->data());

    glPixelStorei(GL_UNPACK_ROW_LENGTH, rowLength);

    if (!_saveImages)
        return;

    auto stem = _sceneRefs.filepath.stem().string() + "_rt_cuda";
    auto f = _ImageFilename(stem, "png");
    int rc = stbi_write_png(f.c_str(), w, h, 4, _frame->data(), _frame->stride() * sizeof (rt::Frame::Pixel));
    if (rc == 0)
        log::error("Failed to write image \"{}\": {}", f, stbi_failure_reason());
    else
        log::info("Image written to \"{}\"", f);
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
void MainWindow::openSceneMenu(const char* label)
{
    if (ImGui::BeginMenu(label, !_sceneFileList.empty()))
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
    switch (type)
    {
    case GL_FLOAT:                  return "FLOAT";
    case GL_INT:                    return "INT";
    case GL_UNSIGNED_INT:           return "UNSIGNED INT";
    case GL_SIGNED_NORMALIZED:      return "SIGNED NORMALIZED";
    case GL_UNSIGNED_NORMALIZED:    return "UNSIGNED NORMALIZED";
    default:                        return "<unknown>";
    }
}

static
const char* framebufferStatusString(GLenum status)
{
    switch (status)
    {
    case GL_FRAMEBUFFER_COMPLETE:
        return "Complete";
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
        return "Incomplete attachment";
    case GL_FRAMEBUFFER_UNDEFINED:
        return "Undefined";
    case GL_FRAMEBUFFER_UNSUPPORTED:
        return "Unsupported";
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
        return "Incomplete Draw buffer";
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
        return "Incomplete Read buffer";
    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
        return "Incomplete Multisample";
    case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
        return "Incomplete layer targets";
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
        return "Incomplete (missing attachment)";
    default:
        return "Incomplete";
    };
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
                auto id = (ImTextureID)selected->handle();
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

    openSceneMenu("Open Scene");
    ImGui::Checkbox("Render on central node", &_state.renderOnCentralNode);
    ImGui::Separator();

    auto& vp = _state.renderViewport;
    ImGui::Text("Render viewport: %dx%d", vp.w, vp.h);

    if (_image != nullptr)
        ImGui::Text("Image of (%d, %d).", _image->width(), _image->height());
    else
        ImGui::Text("No image.");

    if (ImGui::Button("Render (CPU)"))
    {
        _viewMode = ViewMode::Renderer;
        _renderMethod = eCPU;
        renderScene();
        _state.backgroundTaskStatusWindow = &MainWindow::cpuRayTracerStatusWindow;
    }
    ImGui::SameLine();
    if (ImGui::Button("Render (CUDA)"))
    {
        _viewMode = ViewMode::Renderer;
        _renderMethod = eCUDA;
        renderScene();
        _state.backgroundTaskStatusWindow = &MainWindow::cudaRayTracerStatusWindow;
    }
    ImGui::Checkbox("Save image to file", &_saveImages);

    ImGui::SeparatorText("CPU Options");

    static const int maxT = std::thread::hardware_concurrency();
    ImGui::SliderInt("Threads", &_cpuRTOptions.threads, 1, maxT);
    ImGui::DragInt("Number of Samples", &_cpuRTOptions.nSamples, 1.0f, 1, 2048);
    ImGui::DragInt("Recursion Depth", &_cpuRTOptions.recursionDepth, 1.0f, 1, 64);

    if (ImGui::CollapsingHeader("Benchmark/Test (blocking)"))
    {
        ImGui::DragInt("Number of tests", &_testOptions.count, 1.0f, 1, 32);
        ImGui::DragInt("Wait time between tests (ms)",
            &_testOptions.cooldown, 1.0f, 0, 1000);

        using namespace std::chrono_literals;
        using std::chrono::steady_clock;
        static steady_clock::time_point lastBlock = steady_clock::now();

        auto t = std::chrono::duration_cast<std::chrono::milliseconds>
            (steady_clock::now() - lastBlock);

        // Blocking tasks
        if (ImGui::Button("Render Test CPU"))
        {
            if (t > 500ms)
            {
                renderTestCPU();
                lastBlock = steady_clock::now();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Render Test CUDA"))
        {
            if (t > 500ms)
            {
                renderTestCUDA();
                lastBlock = steady_clock::now();
            }
        }
    }

    ImGui::SeparatorText("Additional Info");

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

    ImGui::Text("Window framebuffer: %dx%d",
        framebufferWidth, framebufferHeight);

    ImGui::SeparatorText("Extra Options");

    static float depthValue = 0.99999f;
    ImGui::DragFloat("Environment Depth", &depthValue, 0.0001);
    auto p = editor()->__environmentProg();
    p->disuse();
    p->use();
    p->setDepth(depthValue);

    ImGui::Separator();
    constexpr int maxValue = heatMaxValue;
    for (int i = 0; i <= maxValue; i++)
    {
        if (i > 0)
            ImGui::SameLine(0, 0);
        auto label = std::format("Value ({})", i);
        const auto color = heatPallete(i);
        ImGui::ColorButton(label.c_str(), *(ImVec4*)&color.x);
    }

    ImGui::Separator();
    ImGui::DragFloatRange2("Clip U", &_debugClip[0].x, &_debugClip[0].y, 0.001f);
    ImGui::DragFloatRange2("Clip V", &_debugClip[1].x, &_debugClip[1].y, 0.001f);

    if (ImGui::Button("Reset Clip"))
    {
        _debugClip[1] = _debugClip[0] = {0, 1};
    }

    if (ImGui::Checkbox("Use matrix version", &_matClip))
    {

    }

    if (auto ptr = _sceneRefs.debugObject)
    {
        ImGui::Separator();
        auto t = ptr->transform();
        float scale = std::log10(t->localScale().x);
        float slidermin = -3;
        float slidermax = 20;
        if (ImGui::SliderScalar("Debug Obj Zoom", ImGuiDataType_Float, &scale,
            &slidermin, &slidermax))
        {
            t->setLocalScale(std::pow(10, scale));
        }
    }

#if SPL_BC_STATS
    ImGui::Separator();
    char label[128];
    const auto& patches = spline::stats::g_BezierClippingData;
    ImGui::Text("Patch intersection test data (%zu)", patches.size());
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

void MainWindow::inspectLight(MainWindow& window, graph::LightProxy& proxy)
{
    auto light = proxy.light();
    bool on = light->isTurnedOn();
    ImGui::Checkbox("Turned On", &on);
    light->turnOn(on);

    constexpr const char* types[] {"Directional", "Point", "Spot"};
    auto type = (int) light->type();
    if (ImGui::BeginCombo("Type", types[type]))
    {
        for (int i = 0; i < std::size(types); i++)
        {
            bool selected = i == type;
            if (ImGui::Selectable(types[i], selected))
                type = i;
        }
        ImGui::EndCombo();
    }
    light->setType((Light::Type) type);

    vec3f color {light->color};
    float scale = color.max();
    if (scale > 1)
    {
        color *= 1 / scale;
    }
    else
    {
        scale = 1;
    }
    // NOTE: ImGuiColorEditFlags_HDR is not fully implemented yet.
    ImGui::ColorEdit3("Color", &color.x, ImGuiColorEditFlags_HDR | ImGuiColorEditFlags_Float);
    ImGui::DragFloat("Scale / Strength", &scale, 0.05f, 1.0f, 1000.0f,
        "%.2f", ImGuiSliderFlags_Logarithmic);

    light->color.x = scale * color.x;
    light->color.y = scale * color.y;
    light->color.z = scale * color.z;
    
    ImGui::LabelText("Value", "(%2.2f,%2.2f,%2.2f)",
        light->color.x, light->color.y, light->color.z);

    if (light->type() == Light::Type::Directional)
        return;

    if (light->type() == Light::Type::Point)
    {
        float range = light->range();
        ImGui::DragFloat("Range", &range, 0.1f, 0, 1000);

        bool inf = range == 0;
        ImGui::SameLine();
        ImGui::Checkbox("Infinite", &inf);
        if (inf)
            range = 0;
        light->setRange(range);
        light->flags.enable(Light::Infinite, inf);
    }
    else // Light::Type::Spot
    {
        float angle = light->spotAngle();
        ImGui::DragFloat("Spot Angle", &angle, 0.1f,
            light->minSpotAngle, light->maxSpotAngle);
        light->setSpotAngle(angle);
    }

    constexpr const char* s[] {"Constant", "Linear", "Quadratic"};
    auto falloff = (int) light->falloff;
    if (ImGui::BeginCombo("Falloff", s[falloff]))
    {
        for (int i = 0; i < std::size(s); i++)
        {
            bool selected = i == falloff;
            if (ImGui::Selectable(s[i], selected))
                falloff = i;
        }
        ImGui::EndCombo();
    }
    light->falloff = (Light::Falloff) falloff;
}

void MainWindow::inspectSurface(MainWindow& window, SurfaceProxy& s)
{
    ImGui::inputText("Surface", s.sceneObject()->name());
    auto e = s.mapper()->surface()->surface();
    int a[3] {0};
    int b[3] {0};
    for (auto& g : e->groups())
    {
        if (g.type > 2) continue;
        ++a[(int)g.type];
        b[(int)g.type] += (int)g.count;
    }
    const char* const t[] { "Bézier", "B-Spline", "Gregory" };
    for (int i = 0; i < 3; i++)
    {
        ImGui::LabelText(t[i], "%d groups, %d patches", a[i], b[i]);
    }
    ImGui::Separator();
    window.inspectMaterial(*s.mapper()->surface());
}

void MainWindow::workbenchWindow()
{
    ImGui::Begin("Workbench");

    constexpr const char* items[]
    {
        "Select",
        "Primitive Inspect",
        "Normal Inspect"
    };
    constexpr const char* keybinding[] { "F1", "F2", "F3" };
    constexpr CursorMode modes[]
    {
        Select,
        PrimitiveInspect,
        NormalInspect,
    };

    if (ImGui::CollapsingHeader("Cursor Mode", ImGuiTreeNodeFlags_DefaultOpen))
    {
        for (int i = 0; i < std::size(items); i++)
        {
            bool selected = _state.cursorMode == modes[i];
            if (ImGui::Selectable(items[i], selected,
                ImGuiSelectableFlags_AllowItemOverlap))
                _state.cursorMode = modes[i];
            ImGui::SameLine();
            auto s = keybinding[i];
            auto w = ImGui::CalcTextSize(s).x + 2 * ImGui::GetStyle().FramePadding.x;
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - w);
            ImGui::Text(s);
        }
    }


    ImGui::Separator();

    if (ImGui::Button("Back to Editor"))
        _image = nullptr;
    
    ImGui::SameLine();

    bool hasFile = !_sceneRefs.filepath.empty();
    if (!hasFile) ImGui::BeginDisabled();
    if (ImGui::Button("Reload scene"))
    {
        reloadScene();
    }
    if (!hasFile) ImGui::EndDisabled();

    auto& selectedItem = _image;
    for (auto& [k, item] : _workbench2D)
    {
        bool selected = selectedItem == item;
        if (ImGui::Selectable(k.c_str(), selected))
        {
            selectedItem = selected ? nullptr : item;
        }
    }

    // bool open = true;
    // if (selectedItem)
    // {
    //     auto title = std::format("Texture ({},{})###Texture",
    //         selectedItem->width(), selectedItem->height());
    //     if (ImGui::Begin(title.c_str(), &open))
    //     {
    //         auto id = reinterpret_cast<ImTextureID>((uintptr_t) selectedItem->handle());
    //         auto max = ImGui::GetWindowContentRegionMax();
    //         auto min = ImGui::GetWindowContentRegionMin();
    //         ImGui::Image(id, {max.x - min.x, max.y - min.y});
    //     }
    //     ImGui::End();
    // }

    // if (!open)
    //     selectedItem = nullptr;

    ImGui::End();
}

} // namespace cg
