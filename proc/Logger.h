// Logger.h - Thread-safe singleton logger with console + file output.
// Two ways to log:
//   * log(level, std::string)      - a message that is already composed
//   * info/warn/error/debug(fmt,)  - printf-style, the common case here since
//                                    the rest of the code uses printf formatting
//
// The log file holds exactly ONE run: init() *replaces* the file instead of
// appending to it, so what is on disk is always the last run and nothing else.
// (Pass append = true if an accumulating log is wanted after all.)
//
// On top of the plain levels the logger numbers the pipeline steps, so the file
// reads as a checklist of what the run actually did:
//   2026-08-27 10:11:12 [INFO] ==> [3/10] Reading the ENVI header
//   2026-08-27 10:11:12 [INFO]     samples 1024, lines 8000, bands 128
#pragma once
#include <string>
#include <fstream>
#include <mutex>
#include <cstdarg>

// Let GCC / Clang type-check the printf-style format strings and arguments.
// gnu_printf understands %zu / %lld etc.; on MinGW the plain 'printf' archetype
// maps to the old msvcrt checker that rejects them (the ANSI stdio runtime used
// at run time supports them fine), so prefer gnu_printf where available.
#if defined(__GNUC__)
#  define LOGGER_PRINTF_FMT(fmtIdx, argIdx) __attribute__((format(gnu_printf, fmtIdx, argIdx)))
#else
#  define LOGGER_PRINTF_FMT(fmtIdx, argIdx)
#endif

class Logger {
public:
    enum class LogLevel {
        Debug = 0,
        Info,
        Warning,
        Error
    };

    static Logger& getInstance();

    // Open the log file, REPLACING whatever it held: one run, one log.
    // append = true keeps the previous content instead. Console output works
    // even when no file could be opened.
    bool init(const std::string& logFile, bool append = false);
    void close();

    // Path of the open log file; empty when logging to the console only.
    std::string logPath();

    // Messages below this level are dropped. Default: Info (Debug is hidden).
    void setLevel(LogLevel level);
    LogLevel level();
    // Turn the stderr echo on/off (the file, if open, always gets the line).
    void setConsole(bool enabled);

    // Log an already-composed message (safe for arbitrary text, no formatting).
    void log(LogLevel level, const std::string& message);
    // printf-style sink.
    void logf(LogLevel level, const char* fmt, ...) LOGGER_PRINTF_FMT(3, 4);

    // Convenience methods. The printf-style overload is picked for string
    // literals / format strings; the std::string overload for already-built
    // messages (e.g. concatenations), which are logged verbatim.
    void debug(const char* fmt, ...) LOGGER_PRINTF_FMT(2, 3);
    void info (const char* fmt, ...) LOGGER_PRINTF_FMT(2, 3);
    void warn (const char* fmt, ...) LOGGER_PRINTF_FMT(2, 3);
    void error(const char* fmt, ...) LOGGER_PRINTF_FMT(2, 3);

    void debug(const std::string& msg) { log(LogLevel::Debug,   msg); }
    void info (const std::string& msg) { log(LogLevel::Info,    msg); }
    void warn (const std::string& msg) { log(LogLevel::Warning, msg); }
    void error(const std::string& msg) { log(LogLevel::Error,   msg); }

    // ---- step banners -------------------------------------------------
    // step() announces the next stage of the pipeline; the logger keeps the
    // running number so the caller never has to renumber anything when a stage
    // is inserted. Anything logged between two step() calls is the detail of
    // the step above it and is indented with detail().
    void setStepTotal(int total);   // 0 = unknown -> "[step 3]" instead of "[3/10]"
    void resetSteps();              // start over at 1 (a second sub-command in one run)
    void step(const char* fmt, ...) LOGGER_PRINTF_FMT(2, 3);
    int  currentStep();

    // An indented detail line under the current step. Same levels as above.
    void detail(const char* fmt, ...) LOGGER_PRINTF_FMT(2, 3);
    void detailDebug(const char* fmt, ...) LOGGER_PRINTF_FMT(2, 3);

    ~Logger();

private:
    Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // Format into a std::string with vsnprintf, then hand off to log().
    void vlog(LogLevel level, const char* fmt, va_list ap);
    // Same, but the message is prefixed with `head` after formatting.
    void vlogPrefixed(LogLevel level, const char* head, const char* fmt, va_list ap);

    static std::string vformat(const char* fmt, va_list ap);

    std::ofstream m_logFile;
    std::string   m_path;
    std::mutex    m_mutex;
    LogLevel      m_minLevel = LogLevel::Info;
    bool          m_console  = true;
    int           m_step      = 0;
    int           m_stepTotal = 0;

    static std::string getTimestamp();
    static const char* levelToString(LogLevel level);
};
