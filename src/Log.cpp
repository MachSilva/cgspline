#include "Log.h"

namespace cg::log
{

Ref<Logger> default_logger = new StdLog{};

std::string_view getSeverityString(Severity s)
{
    switch (s)
    {
    case Severity::eDebug:
        return "debug";
    case Severity::eInfo:
        return "info";
    case Severity::eWarning:
        return "warning";
    case Severity::eError:
        return "error";
    default:
        return "unknown";
    }
}

static inline char getSeverityLetter(Severity s)
{
    switch (s)
    {
    case Severity::eDebug:
        return 'D';
    case Severity::eInfo:
        return 'I';
    case Severity::eWarning:
        return 'W';
    case Severity::eError:
        return 'E';
    default:
        return '?';
    }
}

std::ostreambuf_iterator<char> StdLog::log(Severity s)
{
    if (s == eInfo)
        return std::ostreambuf_iterator<char>(_stream);

    return std::format_to(
        std::ostreambuf_iterator<char>(_stream),
        "[{:8}] ",
        getSeverityString(s)
    );
}

void StdLog::end()
{
    _stream << '\n';
}

} // namespace cg::log
