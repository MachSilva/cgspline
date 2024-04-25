#pragma once

#include <array>
#include <graphics/Color.h>

namespace cg
{

template<int NSamples = 2>
requires (NSamples > 1 && NSamples < 9)
struct ColorMap
{
    std::array<vec4f,NSamples> colors {};
    std::array<float,NSamples> values {};

    constexpr ColorMap() = default;
    constexpr ColorMap(decltype(colors)&& a_colors, decltype(values)&& a_values)
        : colors{std::move(a_colors)}, values{std::move(a_values)} {}

    constexpr ColorMap(decltype(colors)&& a_colors, float maxValue = 1.0f)
        : colors{std::move(a_colors)}
    {
        for (int i = 0; i < NSamples; i++)
            values[i] = maxValue * i / float(NSamples - 1);
    }

    constexpr auto size() const { return NSamples; }

    constexpr vec4f operator() (float t) const
    {
        if (t <= values[0])
            return colors[0];
        for (int i = 1; i < NSamples; i++)
        {
            if (t <= values[i])
            {
                auto a = values[i - 1];
                auto b = values[i];
                auto u = (t - a) / (b - a);
                return (1 - u) * colors[i - 1] + u * colors[i];
            }
        }
        return colors.back();
    }
};

} // namespace cg
