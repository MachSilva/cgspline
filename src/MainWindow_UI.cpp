#include "MainWindow.h"

#include <imgui_internal.h>

namespace cg
{

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
    workbenchWindow();
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

    if (ImGui::Button("Render (CPU)"))
    {
        _viewMode = ViewMode::Renderer;
        _renderMethod = eCPU;
        renderScene();
    }
    ImGui::SameLine();
    if (ImGui::Button("Render (CUDA)"))
    {
        _viewMode = ViewMode::Renderer;
        _renderMethod = eCUDA;
        renderScene();
    }

    static const int maxT = std::thread::hardware_concurrency();
    ImGui::SliderInt("CPU threads", &_cpuRTOptions.threads, 1, maxT);

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

void MainWindow::inspectSurface(MainWindow& window, SurfaceProxy& s)
{
    ImGui::inputText("Surface", s.sceneObject()->name());
    ImGui::Separator();
    window.inspectMaterial(s.mapper()->surface());
}

void MainWindow::workbenchWindow()
{
    ImGui::Begin("Workbench");

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
