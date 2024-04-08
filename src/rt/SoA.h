#pragma once

#include <memory_resource>
#include <stdexcept>
#include <tuple>
#include <cuda_runtime.h>

namespace cg::rt
{

namespace detail
{

template<typename index_t, typename... Args>
struct SoABase
{
    std::pmr::memory_resource* _resource;

    SoABase(std::pmr::memory_resource* mr) : _resource{mr} {}

    auto resource() const { return _resource; }

    constexpr void allocate(size_t) {}
    constexpr void deallocate(size_t) {}
    constexpr void construct(size_t) {}
    constexpr void destroy(size_t) {}
    constexpr std::tuple<> as_tuple() const { return {}; }
    constexpr std::tuple<> as_tuple() { return {}; }
    // constexpr void get(size_t) {}
    // constexpr void set(size_t) {}
};

template<typename index_t, typename T, typename... Args>
struct SoABase<index_t, T, Args...> : SoABase<index_t, Args...>
{
    using base_type = SoABase<index_t, Args...>;
    using value_type = T;
    using index_type = index_t;
    using allocator_type = std::pmr::polymorphic_allocator<T>;

    static_assert(std::is_default_constructible_v<T>,
        "SoA: handling for non default constructible types not implemented yet");
    static_assert(std::is_trivially_destructible_v<T>,
        "SoA: handling for non trivially destructible types not implemented yet");

    T* data {};

    SoABase(std::pmr::memory_resource* mr) : base_type{mr} {}

    allocator_type allocator() const
    {
        return allocator_type(base_type::resource());
    }

    void allocate(index_type n)
    {
        data = allocator().allocate(n);
        base_type::allocate(n);
    }

    void deallocate(index_type n)
    {
        allocator().deallocate(data, n);
        data = nullptr;
        base_type::deallocate(n);
    }

    void construct(index_type n)
    {
        std::construct_at(&data[n]);
        base_type::construct(n);
    }

    void destroy(index_type n)
    {
        std::destroy_at(&data[n]);
        base_type::destroy(n);
    }

    std::tuple<const T*, const Args*...> as_tuple() const
    {
        return std::tuple_cat(std::tuple<const T*>(data), base_type::as_tuple());
    }

    std::tuple<T*, Args*...> as_tuple()
    {
        return std::tuple_cat(std::tuple<T*>(data), base_type::as_tuple());
    }

    std::tuple<const T&, const Args&...> get(index_type i) const
    {
        return std::tuple_cat(std::tuple<const T&>(data[i]), base_type::get(i));
    }

    std::tuple<T&, Args&...> get(index_type i)
    {
        return std::tuple_cat(std::tuple<T&>(data[i]), base_type::get(i));
    }

    void set(index_type i, const T& t, const Args&... args)
    {
        data[i] = t;
        base_type::set(i, args...);
    }
};

template<size_t I, typename index_t, typename... Args> struct SoAData
{
    static_assert(I < sizeof...(Args), "SoA: invalid array index");
};

template<typename index_t, typename T, typename... Args>
struct SoAData<0, index_t, T, Args...>
{
    using value_type = T;

    __host__ __device__
    static T* const& get(const SoABase<index_t, T, Args...>& s)
    {
        return s.data;
    }

    __host__ __device__
    static T*& get(SoABase<index_t, T, Args...>& s)
    {
        return s.data;
    }
};

template<size_t I, typename index_t, typename T, typename... Args>
struct SoAData<I, index_t, T, Args...>
{
    using value_type = typename SoAData<I - 1, index_t, Args...>::value_type;

    __host__ __device__
    static auto& get(const SoABase<index_t, T, Args...>& s)
    {
        return SoAData<I - 1, index_t, Args...>
            ::get((const SoABase<index_t, Args...>&) s);
    }

    __host__ __device__
    static auto& get(SoABase<index_t, T, Args...>& s)
    {
        return SoAData<I - 1, index_t, Args...>
            ::get((SoABase<index_t, Args...>&) s);
    }
};

} // namespace detail

template<typename index_t, typename... Args>
struct SoA
{
    static constexpr auto array_count = sizeof...(Args);

    using base_type = detail::SoABase<index_t, Args...>;
    using memory_resource = std::pmr::memory_resource;

    template<index_t I>
    using element_type = typename detail::SoAData<I,Args...>::value_type;

    SoA(index_t capacity = 64,
        memory_resource* mr = std::pmr::get_default_resource())
        : _capacity{capacity}, _data{mr}
    {
        _data.allocate(_capacity);
    }

    ~SoA()
    {
        _data.deallocate(_capacity);
    }

    auto resource() const { return _data.resource(); }
    __host__ __device__ auto capacity() const { return _capacity; }
    __host__ __device__ auto size() const { return _size; }

    template<index_t I>
    __host__ __device__ auto& size_bytes() const noexcept
    {
        return _size * sizeof (element_type<I>);
    }

    void emplace_back()
    {
#ifndef NDEBUG
        if (size() == capacity())
            throw std::length_error("SoA: max capacity exceeded");
#endif
        _data.construct(_size++);
    }

    void push_back(const Args&... args)
    {
#ifndef NDEBUG
        if (size() == capacity())
            throw std::length_error("SoA: max capacity exceeded");
#endif
        _data.set(_size, args...);
        ++_size;
    }

    void pop_back()
    {
#ifndef NDEBUG
        if (size() == 0)
            throw std::length_error(
                "SoA: cannot remove anything from an already empty array");
#endif
        _data.destroy(--_size);
    }

    std::tuple<const Args*...> as_pointer_tuple() const
    {
        return _data.as_tuple();
    }

    std::tuple<const Args&...> get(index_t i) const
    {
#ifndef NDEBUG
        range_check(i);
#endif
        return _data.get(i);
    }

    std::tuple<Args&...> get(index_t i)
    {
#ifndef NDEBUG
        range_check(i);
#endif
        return _data.get(i);
    }

    template<index_t I>
    __host__ __device__
    auto& get(index_t i) const
    {
#if !defined(NDEBUG) && !defined(__NVCC__)
        range_check(i);
#endif
        return data<I>()[i];
    }

    template<index_t I>
    __host__ __device__
    auto& get(index_t i)
    {
#if !defined(NDEBUG) && !defined(__NVCC__)
        range_check(i);
#endif
        return data<I>()[i];
    }

    template<index_t I>
    __host__ __device__
    auto& data() const { return detail::SoAData<I,index_t,Args...>::get(_data); }

protected:
    template<index_t I>
    __host__ __device__
    auto& data() { return detail::SoAData<I,index_t,Args...>::get(_data); }

    template<index_t I>
    auto allocator() const
    {
        return std::pmr::polymorphic_allocator<element_type<I>>(_data.resource());
    }

#ifndef NDEBUG
    void range_check(index_t i) const
    {
        if (i >= size())
            throw std::range_error("SoA: invalid element index");
    }
#endif

    base_type _data;
    index_t _capacity = 0;
    index_t _size = 0;

    friend void testfn();
};

} // namespace cg::rt
