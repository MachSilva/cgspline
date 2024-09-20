//[]---------------------------------------------------------------[]
//|                                                                 |
//| Copyright (C) 2007, 2023 Paulo Pagliosa.                        |
//|                                                                 |
//| This software is provided 'as-is', without any express or       |
//| implied warranty. In no event will the authors be held liable   |
//| for any damages arising from the use of this software.          |
//|                                                                 |
//| Permission is granted to anyone to use this software for any    |
//| purpose, including commercial applications, and to alter it and |
//| redistribute it freely, subject to the following restrictions:  |
//|                                                                 |
//| 1. The origin of this software must not be misrepresented; you  |
//| must not claim that you wrote the original software. If you use |
//| this software in a product, an acknowledgment in the product    |
//| documentation would be appreciated but is not required.         |
//|                                                                 |
//| 2. Altered source versions must be plainly marked as such, and  |
//| must not be misrepresented as being the original software.      |
//|                                                                 |
//| 3. This notice may not be removed or altered from any source    |
//| distribution.                                                   |
//|                                                                 |
//[]---------------------------------------------------------------[]
//
// OVERVIEW: Parser.h (formerly ReaderBase.h)
// ========
// Class definition for generic parser/reader.
//
// Author: Paulo Pagliosa and contributors
// Last revision: 06/07/2023
// Altered version last revision: 09/08/2024

#pragma once

#include <any>
#include <filesystem>
#include <format>
#include <iostream>
#include <map>
#include <vector>
#include "../Ref.h"

namespace cg::util
{ // begin namespace cg::util

class parse_error : public std::runtime_error { using runtime_error::runtime_error; };

class Parser : public SharedObject
{
public:
    struct Source
    {
        std::istream* in = nullptr;
        std::string name;
        std::filesystem::path workingDir;
        int line = 1;
        int column = 1;
    };

    void parse(std::string_view name, std::istream& input,
        std::filesystem::path workingDir = std::filesystem::current_path())
    {
        _sources.clear();
        include(name, input, workingDir);
    }

    std::filesystem::path workingDir() const
    {
        if (_sources.empty())
            return {};
        return _sources.back().workingDir;
    }

protected:
    std::vector<Source> _sources;

    int _token = -1;

    void include(std::string_view name, std::istream& input,
        std::filesystem::path workingDir);

    // peek char from last stream
    int peek()
    {
        return _sources.back().in->peek();
    }

    // get char from last stream
    int get()
    {
        auto& s = _sources.back();
        int c = s.in->get();
        if (c == 10)
        {
            s.column = 1;
            s.line++;
        }
        else
        {
            s.column++;
        }
        return c;
    }

    virtual void doParse();
    virtual int nextToken();

    int next()
    {
        return _token = nextToken();
    }

    template<typename... Ts>
    [[noreturn]] void
    _e(std::format_string<Ts...> fmt, Ts&&... ts) const
    {
        error(std::format(fmt, std::forward<Ts>(ts)...));
    }

    [[noreturn]] void error(std::string_view msg) const;
};

class BasicLanguageParser : public Parser
{
protected:
    enum
    {
        _IDENTIFIER = 0x1000000,
        _INTEGER,
        _FLOAT,
        _STRING,
        lastToken
    };

    struct Keyword
    {
        int token {};
        std::any value {};
    };

    std::string _lexeme;
    std::any _value;

    void append(char c)
    {
        _lexeme.append(1, c);
    }

    void doParse() override;
    int nextToken() override;

    virtual const Keyword* findKeywordToken(const std::string&) const;

    void match(int token);
    std::string matchIdentifier();
    std::string matchString();
    float matchFloat();
    int matchInteger();
};

} // end namespace cg::util
