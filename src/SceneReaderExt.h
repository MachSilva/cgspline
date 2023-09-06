#pragma once

#include <reader/SceneReader.h>
#include "Surface.h"

namespace cg::util
{

class SceneReaderExt : public SceneReader
{
public:
    class Parser;

    parser::Reader::Parser* makeParser() override;

protected:
    using Base = SceneReader;
};

class SceneReaderExt::Parser : public SceneReader::Parser
{
public:
    Parser(SceneReaderExt& reader)
        : SceneReader::Parser{reader}
        , _reader{reader} {}

protected:
    SceneReaderExt& _reader;

    using Base = SceneReader::Parser;

    enum // ComponentTypes
    {
        _SURFACE = Base::lastComponentType,
        lastComponentType
    };

    enum // Tokens
    {
        lastToken = Base::lastToken
    };

    enum // ErrorCodes
    {
        COULD_NOT_LOAD_SURFACE = Base::lastErrorCode,
        lastErrorCode
    };

    Reference<graph::Component>
    parseComponent(int, graph::SceneObject&) override;

    Reference<SurfaceProxy> matchSurface();

    DECLARE_ERROR_MESSAGE_TABLE(SceneReaderExt::Parser);
    DECLARE_KEYWORD_TABLE(SceneReaderExt::Parser);
};

} // namespace cg::util

