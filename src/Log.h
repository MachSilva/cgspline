#pragma once

#include <format>
#include <iostream>
#include <thread>
#include "Ref.h"

namespace cg::log
{

enum Severity : int
{
    eDebug,
    eInfo,
    eWarning,
    eError
};

std::string_view getSeverityString(Severity);

class Logger : public virtual SharedObject
{
protected:
    std::ostream _stream;

    Logger(std::streambuf* buf) : _stream(buf) {}
    Logger(Logger&&) = delete;
    Logger(const Logger&) = delete;

public:
    virtual std::ostreambuf_iterator<char> log(Severity) = 0;
    virtual void end() = 0;

    std::streambuf* buffer() noexcept
    {
        return _stream.rdbuf();
    }
};

extern Ref<Logger> default_logger;

template<typename... Args>
inline void debug(std::format_string<Args...> fmt, Args&&... args)
{
    auto it = default_logger->log(Severity::eDebug);
    std::format_to(it, fmt, std::forward<Args>(args)...);
    default_logger->end();
}

template<typename... Args>
inline void info(std::format_string<Args...> fmt, Args&&... args)
{
    auto it = default_logger->log(Severity::eInfo);
    std::format_to(it, fmt, std::forward<Args>(args)...);
    default_logger->end();
}

template<typename... Args>
inline void warn(std::format_string<Args...> fmt, Args&&... args)
{
    auto it = default_logger->log(Severity::eWarning);
    std::format_to(it, fmt, std::forward<Args>(args)...);
    default_logger->end();
}

template<typename... Args>
inline void error(std::format_string<Args...> fmt, Args&&... args)
{
    auto it = default_logger->log(Severity::eError);
    std::format_to(it, fmt, std::forward<Args>(args)...);
    default_logger->end();
}

// Logger classes

class StdLog : public Logger
{
public:
    StdLog(std::streambuf* buf = std::clog.rdbuf()) : Logger(buf) {}

    std::ostreambuf_iterator<char> log(Severity) override;
    void end() override;
};

// streambuf classes

class replicate : public std::streambuf
{
public:
    explicit replicate() {}

    // TODO ...
};

} // namespace cg::log
