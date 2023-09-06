#include "SceneReaderExt.h"

#include <graphics/Application.h>

/*
Grammar Extension (refer to "../Ds/apps/cgdemo/reader/grammar.txt").

Redefinition of Component token.

Component:
  Camera
| Light
| Primitive
| Surface

Surface:
  SURFACE STRING<:filename> (MATERIAL STRING)?

*/

namespace cg::util
{

DEFINE_KEYWORD_TABLE(SceneReaderExt::Parser, SceneReader::Parser)
    KEYWORD("surface", _COMPONENT, _SURFACE)
END_KEYWORD_TABLE;

DEFINE_ERROR_MESSAGE_TABLE(SceneReaderExt::Parser, SceneReader::Parser)
    ERROR_MESSAGE(COULD_NOT_LOAD_SURFACE,
        "Could not load surface '%s'")
END_ERROR_MESSAGE_TABLE;

parser::Reader::Parser* SceneReaderExt::makeParser()
{
    return new SceneReaderExt::Parser(*this);
}

Reference<graph::Component>
SceneReaderExt::Parser::parseComponent(int type, graph::SceneObject& obj)
{
    switch (type)
    {
    case _SURFACE:
    {
        auto s = matchSurface();
        return (graph::Component*) s;
    }
    
    default:
        return Base::parseComponent(type, obj);
    }
}

Reference<SurfaceProxy>
SceneReaderExt::Parser::matchSurface()
{
    auto filename = std::filesystem::path(matchString());

    if (filename.empty())
        error(EMPTY_FILENAME);

    Reference<SurfaceProxy> component {nullptr};

    auto path = !filename.is_absolute()
        ? _reader._currentPath / filename
        : filename;
    auto patches = BezierPatches::load(path.c_str()); 

    if (!patches)
        error(COULD_NOT_LOAD_SURFACE, filename.c_str());
    else
    {
        component = new SurfaceProxy(patches.get());
        parsePrimitiveMaterial(component->mapper()->surface());
    }

    return component;
}

} // namespace cg::util

