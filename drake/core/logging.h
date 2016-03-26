#ifndef DRAKE_CORE_LOGGING_H_
#define DRAKE_CORE_LOGGING_H_

// TODO(jeremy.nimmer): Spell this however cmake wants us to.
#ifdef HAVE_SPDLOG

namespace Drake {

std::shared_ptr<logging::logger> log();
// In logging.cc, implement Drake::log() via spdlog::get("console"),
// but if null then latch-init with stderr_logger_st("console").

namespace logging {
using spdlog::logger;
using spdlog::details::line_logger
}
} // namespace Drake

#else  // HAVE_SPDLOG

namespace Drake {
namespace logging {

// A no-nothing line_logger stub.
class line_logger {
 public:
  template <typename T>
  inline line_logger& operator<<(T) {}
};

// A do-nothing logger stub.
class logger {
 public:
  details::line_logger trace() const { return line_logger(); }
  details::line_logger debug() const { return line_logger(); }
  details::line_logger info() const { return line_logger(); }
  details::line_logger notice() const { return line_logger(); }
  details::line_logger warn() const { return line_logger(); }
  details::line_logger error() const { return line_logger(); }
  details::line_logger critical() const { return line_logger(); }
  details::line_logger alert() const { return line_logger(); }
  details::line_logger emerg() const { return line_logger(); }
};

// A do-nothing logger instance.
namespace detail { const logger g_logger; }

} // namespace logging

const logging::logger* log() { return &g_logger; }

} // namespace Drake

#define SPDLOG_TRACE(logger, ...)
#define SPDLOG_DEBUG(logger, ...)

#endif  // HAVE_SPDLOG

#endif
