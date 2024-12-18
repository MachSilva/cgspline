#include "DescriptionLanguage.h"

#include <typeinfo>

namespace cg::util
{

using Dict = DescriptionLanguage::Dict;
using List = DescriptionLanguage::List;
using Value = DescriptionLanguage::Value;

[[noreturn]]
static void _operatorTypeMismatch(std::string_view function,
    const Value& t1, const Value& t2)
{
    throw std::invalid_argument(_f(
        "operator `{}` cannot be called with argument types `{}` and `{}`",
        function, t1.type_name(), t2.type_name()));
}

template<typename T, typename... Types>
concept is_one_of = (std::same_as<T,Types> || ...);

template<typename T>
concept is_vector = is_one_of<T,vec2f,vec3f,vec4f>;

Value _add(const List& args)
{
    if (args.size() < 2)
        throw std::runtime_error("expected at least 2 arguments");

    using enum Value::Type;
    using std::get;
    constexpr auto op = "+";
    auto a = args[0];
    for (int i = 1 ; i < args.size(); i++)
    {
        auto& b = args[i];
        switch (a.index())
        {
        case eInt:
            if (b.index() == eInt)
                a = get<eInt>(a) + get<eInt>(b);
            else if (b.index() == eFloat)
                a = float(get<eInt>(a)) + get<eFloat>(b);
            else
                _operatorTypeMismatch(op, a, b);
            break;
        case eFloat:
            if (b.index() == eInt)
                a = get<eFloat>(a) + float(get<eInt>(b));
            else if (b.index() == eFloat)
                a = get<eFloat>(a) + get<eFloat>(b);
            else
                _operatorTypeMismatch(op, a, b);
            break;
        case eVec2:
            if (b.index() == eVec2)
                a = get<eVec2>(a) + get<eVec2>(b);
            else
                _operatorTypeMismatch(op, a, b);
            break;
        case eVec3:
            if (b.index() == eVec3)
                a = get<eVec3>(a) + get<eVec3>(b);
            else
                _operatorTypeMismatch(op, a, b);
            break;
        case eVec4:
            if (b.index() == eVec4)
                a = get<eVec4>(a) + get<eVec4>(b);
            else
                _operatorTypeMismatch(op, a, b);
            break;
        default:
            _operatorTypeMismatch(op, a, b);
        }

    }
    return a;
}

Value _sub(const List& args)
{
    using enum Value::Type;
    using std::get;
    constexpr auto op = "-";

    const auto n = args.size();
    if (n == 1)
    {
        auto& a = args[0];
        switch (a.index())
        {
        case eInt:
            return -get<eInt>(a);
        case eFloat:
            return -get<eFloat>(a);
        case eVec2:
            return -get<eVec2>(a);
        case eVec3:
            return -get<eVec3>(a);
        case eVec4:
            return -get<eVec4>(a);
        default:
            throw std::invalid_argument(_f(
                "operator `-` not supported with type `{}`",
                a.type_name()));
        }
    }
    else if (n == 2)
    {
        auto& a = args[0];
        auto& b = args[1];
        switch (a.index())
        {
        case eInt:
            if (b.index() == eInt)
                return get<eInt>(a) - get<eInt>(b);
            if (b.index() == eFloat)
                return float(get<eInt>(a)) - get<eInt>(b);
            break;
        case eFloat:
            if (b.index() == eInt)
                return get<eFloat>(a) - float(get<eInt>(b));
            if (b.index() == eFloat)
                return get<eFloat>(a) - get<eFloat>(b);
            break;
        case eVec2:
            if (b.index() == eVec2)
                return get<eVec2>(a) - get<eVec2>(b);
            break;
        case eVec3:
            if (b.index() == eVec3)
                return get<eVec3>(a) - get<eVec3>(b);
            break;
        case eVec4:
            if (b.index() == eVec4)
                return get<eVec4>(a) - get<eVec4>(b);
            break;
        }
        _operatorTypeMismatch(op, a, b);
    }
    else throw std::invalid_argument("expected one or two arguments");
    // return {};
}

const float* to_float(const Value* a)
{
    thread_local float tmp;
    if (auto p = std::get_if<int>(a))
        return &(tmp = *p);
    return std::get_if<float>(a);
};

Value _mul(const List& args)
{
    if (args.size() < 2)
        throw std::runtime_error("expected at least 2 arguments");

    using enum Value::Type;
    using std::get;
    constexpr auto op = "*";

    auto a = args[0];
    for (int i = 1 ; i < args.size(); i++)
    {
        auto& b = args[i];

        a = std::visit([](auto a, auto b) -> Value
        {
            using A = std::decay_t<decltype(a)>;
            using B = std::decay_t<decltype(b)>;
            if constexpr (std::is_scalar_v<A> && std::is_scalar_v<B>)
                return a * b;
            else if constexpr (std::is_scalar_v<A> && is_vector<B>)
                return (typename B::value_type)(a) * b;
            else if constexpr (std::is_scalar_v<B> && is_vector<A>)
                return a * (typename A::value_type)(b);
            else
                _operatorTypeMismatch(op, a, b);
        }, a, b);
    }
    return a;
}

Value _div(const List& args)
{
    if (args.size() != 2)
        throw std::invalid_argument("expected only two arguments");

    using enum Value::Type;
    using std::get_if;
    auto& a = args[0];
    auto& b = args[1];

    if (auto p = get_if<int>(&a))
    {
        if (auto q = get_if<int>(&b))
            return *p / *q;
        if (auto q = get_if<float>(&b))
            return float(*p) / *q;
        float i = 1.0f / float(*p);
        if (auto q = get_if<vec2f>(&b))
            return i * (*q);
        if (auto q = get_if<vec3f>(&b))
            return i * (*q);
        if (auto q = get_if<vec4f>(&b))
            return i * (*q);
    }
    else if (auto p = get_if<float>(&a))
    {
        if (auto q = get_if<int>(&b))
            return *p / float(*q);
        if (auto q = get_if<float>(&b))
            return *p / *q;
        float i = 1.0f / (*p);
        if (auto q = get_if<vec2f>(&b))
            return i * (*q);
        if (auto q = get_if<vec3f>(&b))
            return i * (*q);
        if (auto q = get_if<vec4f>(&b))
            return i * (*q);
    }
    else if (auto p = get_if<vec2f>(&a))
    {
        if (auto q = to_float(&b))
            return *p * (1.0f / *q);
    }
    else if (auto p = get_if<vec3f>(&a))
    {
        if (auto q = to_float(&b))
            return *p * (1.0f / *q);
    }
    else if (auto p = get_if<vec4f>(&a))
    {
        if (auto q = to_float(&b))
            return *p * (1.0f / *q);
    }
    _operatorTypeMismatch("/", a, b);
    // return {};
}

static Value From_sRGB(const List& args)
{
    const auto n = args.size();
    if (n != 3 || n != 4)
        throw std::runtime_error("From_sRGB function: expected 3 or 4 integer arguments");

    auto component = [](const Value& v) -> int
    {
        int c = std::get<int>(v);
        if (c < 0 || c > 255)
            throw std::out_of_range("From_sRGB component must be between 0 and 255");
        return c;
    };

    int r = component(args[0]);
    int g = component(args[1]);
    int b = component(args[2]);
    int a = n == 4 ? component(args[3]) : 255;

    Color c (r, g, b, a);
    return vec4f((const float*) c);
}

DescriptionLanguage::DescriptionLanguage() : _scope{new Dict}
{
    _scope->insert({"+", (Function) _add});
    _scope->insert({"-", (Function) _sub});
    _scope->insert({"*", (Function) _mul});
    _scope->insert({"/", (Function) _div});
    _scope->insert({"From_sRGB", (Function) From_sRGB});
}

const DescriptionLanguage::Keyword*
DescriptionLanguage::findKeywordToken(const std::string& s) const
{
    static const Keyword table[]
    {
        { .token = _LET }
    };

    if (s == "let")
    {
        return table;
    }
    return nullptr;
}

Value DescriptionLanguage::matchVector()
{
    int n = 0;
    float v[4];
    match('(');
    v[0] = matchFloat(); n++;
    v[1] = matchFloat(); n++;
    if (_token == _INTEGER || _token == _FLOAT)
    {
        v[2] = matchFloat(); n++;
        if (_token == _INTEGER || _token == _FLOAT)
        {
            v[3] = matchFloat(); n++;
        }
    }
    match(')');
    if (n == 2)
        return vec2f(v);
    if (n == 3)
        return vec3f(v);
    return vec4f(v);
}

// Literal <- Vector / FLOAT / INTEGER / STRING
Value DescriptionLanguage::matchLiteral()
{
    if (_token == '(')
        return matchVector();
    if (_token == _INTEGER)
    {
        auto e = std::any_cast<int>(_value);
        next();
        return e;
    }
    if (_token == _FLOAT)
    {
        auto e = std::any_cast<float>(_value);
        next();
        return e;
    }
    if (_token == _STRING)
    {
        auto e = std::move(_lexeme);
        next();
        return e;
    }
    error("expected a constant literal value");
}

// List <- '[' Value* ']'
Ref<List> DescriptionLanguage::matchList()
{
    auto list = Ref(new List());
    match('[');
    while (_token != ']')
    {
        list->emplace_back(*matchValue());
    }
    match(']');
    return list;
}

// Value <- &(IDENTIFIER / Operator) Expr
//      / &'{' Dict
//      / &'[' List
//      / Literal
Value* DescriptionLanguage::matchValue()
{
    _transient.reset();
    switch (_token)
    {
    case _IDENTIFIER:
    case '+':
    case '-':
    case '*':
    case '/':
        return matchExpr();
    case '{':
        return &(_transient = matchDict());
    case '[':
        return &(_transient = matchList());
    // case '(':
    // case _FLOAT:
    // case _INTEGER:
    // case _STRING:
    }
    return &(_transient = matchLiteral());
}

// Property <- (IDENTIFIER / STRING) ':' Value
void DescriptionLanguage::matchProperty(std::string& property, Value*& value)
{
    if (_token != _IDENTIFIER && _token != _STRING)
        error("property definition expected");

    property = std::move(_lexeme);
    next();

    match(':');
    value = matchValue();
}

// Dict <- '{' Property* '}'
Ref<Dict> DescriptionLanguage::matchDict()
{
    auto props = Ref(new Dict);

    match('{');

    while (atProperty())
    {
        std::string key;
        Value* value {};
        matchProperty(key, value);
        
        if (props->contains(key))
            _e("property \"{}\" already defined", key);
        (*props)[key] = std::move(*value);
    }

    match('}');
    return props;
}

// Root <- IDENTIFIER / Operator / PropertyAccess
Value* DescriptionLanguage::matchRoot()
{
    if (_token == _IDENTIFIER)
    {
        auto id = matchIdentifier();
        return access(_scope, id);
    }
    if (atOperator())
    {
        std::string op;
        op = (char) _token;
        next();
        return access(_scope, op);
    }
    if (atPropertyAccess())
    {
        return matchPropertyAccess(_scope);
    }
    error("expected expression root");
}

// PropertyAccess <-
//      '.' IDENTIFIER          # dict property access
//    / '.' STRING
Value* DescriptionLanguage::matchPropertyAccess(Dict* object)
{
    match('.');

    std::string id;
    if (_token == _IDENTIFIER)
        id = matchIdentifier();
    else if (_token == _STRING)
        id = matchString();
    else
        _e("a property name was expected");

    return access(object, id);
}

// FunctionCall <-
//      '(' Value* ')'
//    / Dict
Value* DescriptionLanguage::matchFunctionCall(const Function& f)
{
    List args;
    if (_token == '{')
    {
        args.emplace_back(matchDict());
    }
    // if (_token == '(')
    else
    {
        match('(');
        while (_token != ')')
        {
            args.emplace_back(*matchValue());
        }
        next();
    }

    auto s = _sources.back();
    try
    {
        return &(_transient = f(args));
    }
    catch (const std::exception& e)
    {
        _e("function evaluation failed at {}:{}:{}:\n{}",
            s.name, s.line, s.column, e.what());
    }
}

// Suffix <-
//      &'.' PropertyAccess Suffix?
//    / &('(' / '{') FunctionCall Suffix?
Value* DescriptionLanguage::matchSuffix(Value* prev)
{
    constexpr auto undefined = "undefined value/property";
    if (_token == '.')
    {
        if (!prev)
            _e(undefined);

        auto obj = prev->getSharedObject<Dict>();
        if (!obj)
            _e("expression result is not a dictionary that holds any property");
        prev = matchPropertyAccess(obj);
    }
    else if (_token == '(' || _token == '{')
    {
        if (!prev)
            _e(undefined);

        auto f = std::get_if<Function>(prev);
        if (!f)
            _e("expression result is not a function");
        prev = matchFunctionCall(*f);
    }
    // else if (_token == '=')
    // { // assignment
    //     next();
    // }
    else error("internal logic error: this rule should not have been called");

    if (atSuffix())
    {
        return matchSuffix(prev);
    }
    return prev;
}

// Expr <- Root Suffix
Value* DescriptionLanguage::matchExpr()
{
    auto value = matchRoot();
    return atSuffix() ? matchSuffix(value) : value;
}

// Statement <- (LET IDENTIFIER '=')? Value
Value* DescriptionLanguage::matchStatement()
{
    std::string id;

    if (_token == _LET)
    {
        next();
        id = matchIdentifier();

        if (_scope->contains(id))
            _e("identifier \"{}\" already defined", id);

        match('=');
    }

    if (!id.empty())
    {
        return &((*_scope)[id] = *matchValue());
    }
    return matchValue();
}

// Block <- Statement*
Value* DescriptionLanguage::matchBlock()
{
    lastObject = nullptr;
    while (_token >= 0)
    {
        lastObject = matchStatement();
    }
    return lastObject;
}

// G <- Block
void DescriptionLanguage::doParse()
{
    (void) matchBlock();
}

} // namespace cg::util
