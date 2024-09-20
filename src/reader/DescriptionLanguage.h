#pragma once

#include <math/Vector4.h>
#include <format>
#include <functional>
#include "Parser.h"
#include "../Ref.h"

namespace cg::util
{

// just an alias for std::format
template<typename... Ts>
inline auto _f(std::format_string<Ts...> fmt, Ts&&... ts)
{
    return std::format(fmt, std::forward<Ts>(ts)...);
}

class DescriptionLanguage : public BasicLanguageParser
{
public:
    struct Value;
    struct List : SharedObject, std::vector<Value> { using vector::vector; };
    struct Dict : SharedObject, std::map<std::string,Value>
    {
        using map::map;

        const Value* get_ptr(const std::string& key) const
        {
            auto it = find(key);
            return it != end() ? &it->second : nullptr;
        }

        Value* get_ptr(const std::string& key)
        {
            auto it = find(key);
            return it != end() ? &it->second : nullptr;
        }

        const Value& at(const std::string& key) const
        {
            auto it = find(key);
            if (it == end())
                throw std::runtime_error
                    (_f("property \"{}\" does not exist", key));
            return it->second;
        }

        Value& at(const std::string& key)
        {
            auto it = find(key);
            if (it == end())
                throw std::runtime_error
                    (_f("property \"{}\" does not exist", key));
            return it->second;
        }
    };

    using Function =  std::function<Value(const List&)>;

    struct Value : std::variant<
        int,float,vec2f,vec3f,vec4f,std::string,Ref<SharedObject>,Function
    >
    {
        enum Type { eInt, eFloat, eVec2, eVec3, eVec4, eString, eSharedObject, eFunction };

        using variant::variant;

        static constexpr const char* type_names[] =
        {
            "int", "float", "vec2", "vec3", "vec4", "string", "object", "function"
        };

        const char* type_name() const noexcept
        {
            auto i = index();
            if (i > eFunction)
                return "valueless_by_exception";
            return type_names[i];
        }

        template<std::derived_from<SharedObject> T = SharedObject>
        T* getSharedObject() const
        {
            auto ref = std::get_if<eSharedObject>(this);
            if (!ref)
                return nullptr;
            return dynamic_cast<T*>(ref->get());
        }

        template<typename T>
        T* castTo() const
        {
            auto ptr = getSharedObject<T>();
            if (!ptr)
            {
                throw std::runtime_error(std::format(
                    "expected an object of type `{}`",
                    typeid (T).name()
                ));
            }
            return ptr;
        }

        // Returns both float or an int converted to float
        float getFloat() const
        {
            auto p = std::get_if<int>(this);
            if (p)
                return float(*p);
            return std::get<float>(*this);
        }

        // Returns a vec3 converted to a vec4 or a vec4 representing a color
        vec4f getColor() const
        {
            if (auto p = std::get_if<vec3f>(this))
                return vec4f(*p);
            if (auto p = std::get_if<vec4f>(this))
                return *p;
            throw std::runtime_error("expected a vec3 or a vec4 as a color");
        }

        void reset() noexcept { *this = {}; }
    };

    Value* lastObject = nullptr;

    DescriptionLanguage();

    bool hasStorage(const Value* p) const { return &_transient != p; }
    // bool isConstant(const Value*) const;

protected:
    enum
    {
        _LET = BasicLanguageParser::lastToken,
        lastToken
    };

    Ref<Dict> _scope;
    Value _transient; //< temporary values without storage go here

    const Keyword* findKeywordToken(const std::string&) const override;

    Value* access(Dict* obj, const std::string& name)
    {
        auto p = obj->get_ptr(name);
        if (!p)
            _e("property \"{}\" undefined", name);
        return p;
    }

    // Operator <- '+' / '-' / '*' / '/'
    bool atOperator() const
    {
        auto t = _token;
        return t == '+' || t == '-' || t == '*' || t == '/';
    }

    // Vector <-
    //      '(' FLOAT FLOAT ')'             # vec2
    //    / '(' FLOAT FLOAT FLOAT ')'       # vec3
    //    / '(' FLOAT FLOAT FLOAT FLOAT ')' # vec4
    Value matchVector();

    // Literal <- Vector / FLOAT / INTEGER / STRING
    Value matchLiteral();

    // List <- '[' Value* ']'
    Ref<List> matchList();

    // Value <- Expr / Dict / List / Literal
    Value* matchValue();

    // Property <- (IDENTIFIER / STRING) ':' Value
    void matchProperty(std::string&, Value*&);

    // Dict <- '{' Property* '}'
    Ref<Dict> matchDict();

    // PropertyAccess <-
    //      '.' IDENTIFIER          # dict property access
    //    / '.' STRING
    Value* matchPropertyAccess(Dict*);

    // FunctionCall <-
    //      '(' Value* ')'          # function call
    //    / Dict                    # function call with a dict as first argument
    Value* matchFunctionCall(const Function&);

    // Root <- IDENTIFIER / Operator / PropertyAccess
    Value* matchRoot();

    bool atProperty() const
    {
        return _token == _IDENTIFIER || _token == _STRING;
    }

    bool atPropertyAccess() const
    {
        return _token == '.';
    }

    bool atSuffix() const
    {
        return _token == '{' || _token == '(' || _token == '.';
    }

    // Suffix <-
    //      PropertyAccess Suffix?
    //    / FunctionCall Suffix?
    Value* matchSuffix(Value*);

    // Expr <- Root Suffix
    Value* matchExpr();

    // Statement <- (IDENTIFIER PropertyAccess* '=')? Value
    Value* matchStatement();

    // Block <- Statement*
    Value* matchBlock();

    // G <- Block
    void doParse() override;
};

} // namespace cg::util
