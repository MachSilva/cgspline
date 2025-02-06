#pragma once

#include <core/SharedObject.h>
#include <graph/Scene.h>
#include "DescriptionLanguage.h"
#include "../SurfacePipeline.h"
#include "../PBRMaterial.h"
#include "../Texture.h"

namespace cg
{

class SceneReader : public util::DescriptionLanguage
{
public:
    SceneReader();

    auto assets() const { return _assets; }
    auto scene() const { return _scene; }

    Ref<gl::Texture> environment;
    struct
    {
        vec3f position;
        quatf rotation;
    } view;

protected:
    Value createMaterial(const List&);
    Value createPBRMaterial(const List&);
    Value createMesh(const List&);
    Value createTexture(const List&);
    Value createSurface(const List&);
    Value getAsset(const List&);

    void createSceneObject(const Dict*, graph::SceneObject* parent);

    Ref<graph::PrimitiveProxy> addMesh(const Dict*);
    Ref<SurfaceProxy> addSurface(const Dict*);
    Value readScene(const List&);

    void doParse() override;

    Ref<Dict> _assets;
    Ref<graph::Scene> _scene;
    bool _sceneComplete;
    int _objectId;
    int _materialId;
    int _meshId;
    int _surfaceId;

    auto insertAsset(const std::string& s, SharedObject* obj)
    {
        auto [it, ok] = _assets->insert({s, obj});
        if (!ok)
            _e("asset \"{}\" already exists", s);
        return it;
    }
};

} // namespace cg
