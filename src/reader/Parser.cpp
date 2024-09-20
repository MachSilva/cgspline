//[]---------------------------------------------------------------[]
//|                                                                 |
//| Copyright (C) 2007, 2022 Paulo Pagliosa.                        |
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
// OVERVIEW: Parser.cpp (formerly ReaderBase.cpp)
// ========
// Source file for generic reader base.
//
// Author: Paulo Pagliosa and contributors
// Last revision: 07/02/2022
// Altered version last revision: 09/08/2024

#include "Parser.h"
#include <cassert>
#include <cctype>
#include <cstdlib>

namespace cg::util
{ // begin namespace cg::util

template<typename F>
struct Finalizer
{
    Finalizer(F&& f) : _f{std::move(f)} {}
    ~Finalizer() { _f(); }

    F _f;
};

void Parser::include(std::string_view name, std::istream& input,
    std::filesystem::path workingDir)
{
    bool found = false;
    for (auto& e : _sources)
    {
        if (e.name == name)
        {
            found = true;
            break;
        }
    }
    
    if (!found)
    {
        _sources.push_back({
            .in = &input,
            .name = std::string(name),
            .workingDir = workingDir,
        });
        Finalizer _ ([&]() { _sources.pop_back(); });
        next();
        (void) doParse();
    }
}

void Parser::error(std::string_view msg) const
{
    std::string s;
    s.reserve(1024);
    auto it = std::back_inserter(s);
    for (auto& e : _sources)
    {
        std::format_to(it, "In {}:{}:{}:\n", e.name, e.line, e.column);
    }
    s.append(msg);
    throw parse_error(s);
}

void Parser::doParse()
{
    // empty
}

int Parser::nextToken()
{
    return get();
}

const BasicLanguageParser::Keyword*
BasicLanguageParser::findKeywordToken(const std::string& s) const
{
    return nullptr;
}

int BasicLanguageParser::nextToken()
{
    int c;
    bool isFloat;
    _value.reset();

_whitespace:
    while (isspace(c = get()));
    if (c < 0)
        return c;
    if (c == '/')
    {
        c = peek();
        if (c == '/')
        {
            (void) get();
            while ((c = peek()) != '\n' && c >= 0)
            {
                (void) get();
            }
            goto _whitespace;
        }
        else if (c == '*')
        {
            (void) get();

            while ((c = get()) >= 0)
            {
                if (c == '*' && peek() == '/')
                {
                    (void) get();
                    goto _whitespace;
                }
            }
            error("unexpected end-of-file");
        }
        return '/';
    }
    _lexeme.clear();
    if (isalpha(c) || c == '_')
    {
        append(c);
        while ((c = peek()) == '_' || isalnum(c))
        {
            append(get());
        }
        auto k = findKeywordToken(_lexeme);
        if (k)
        {
            _value = k->value;
            return k->token;
        }
        return _IDENTIFIER;
    }
    if ((c == '+' || c == '-') && isdigit(peek()))
    {
        append(c);
        c = get();
        goto _number;
    }
    if (isdigit(c))
    {
    _number:
        isFloat = false;
        append(c);
        while (isdigit(c = peek()))
        {
            append(get());
        }
        if ((c = peek()) == '.')
        {
            append(get());
        _float:
            isFloat = true;
            while (isdigit(c = peek()))
            {
                append(get());
            }
        }
        if (toupper(c = peek()) == 'E')
        {
            isFloat = true;
            append(get());
            if ((c = peek()) == '+' || c == '-')
                append(get());
            if (isdigit(c = peek()))
            {
                append(get());
                while (isdigit(c = peek()))
                {
                    append(get());
                }
            }
            else
                _e("unexpected char {}", c);
        }
        if (isFloat)
        {
            _value = std::stof(_lexeme);
            return _FLOAT;
        }
        else
        {
            _value = std::stoi(_lexeme);
            return _INTEGER;
        }
    }
    // if (c == '.')
    // {
    //     append(c);
    //     if (isdigit(c = peek()))
    //         goto _float;
    //     _e("unexpected char {}", c);
    // }
    if (c == '"')
    {
        while ((c = get()) != '"')
        {
            if (c < 0 || c == '\n')
                error("missing string literal ending");
            append(c);
        }
        return _STRING;
    }
    return c;
}

void BasicLanguageParser::doParse()
{
    while (next() >= 0) {}
}

void BasicLanguageParser::match(int token)
{
    if (_token == token)
        next();
    else
        error("unexpected char");
}

std::string BasicLanguageParser::matchIdentifier()
{
    if (_token != _IDENTIFIER)
        error("an identifier was expected");

    auto s = std::move(_lexeme);

    next();
    return s;
}

std::string BasicLanguageParser::matchString()
{
    if (_token != _STRING)
        error("a string was expected");

    auto s = std::move(_lexeme);

    next();
    return s;
}

float BasicLanguageParser::matchFloat()
{
    float e;

    if (_token == _FLOAT)
    {
        e = std::any_cast<float>(_value);
    }
    else if (_token == _INTEGER)
    {
        e = (float) std::any_cast<int>(_value);
    }
    else
    {
        error("a floating point number was expected");
    }

    next();
    return e;
}

int BasicLanguageParser::matchInteger()
{
    if (_token != _INTEGER)
        error("an integer number was expected");

    int e = std::any_cast<int>(_value);

    next();
    return e;
}

} // end namespace cg::util
