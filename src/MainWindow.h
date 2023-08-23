#pragma once

#include <graph/SceneWindow.h>
#include <graph/SceneObject.h>
#include "BezierPatches.h"

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
    // void renderScene() override;
    void gui() override;
    // bool onResize(int width, int height) override;
    graph::SceneObject* pickObject(int x, int y) const override;

protected:
    graph::SceneObject* createSurfaceObject(BezierPatches& p, const char* name);
    void drawSelectedObject(const graph::SceneObject& object);

    graph::SceneObject* pickObject(graph::SceneObject* obj,
        const Ray3f& ray,
        float& distance) const;
};

} // namespace cg
