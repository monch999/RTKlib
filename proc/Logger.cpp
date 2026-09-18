// Logger.cpp - implementation of the thread-safe singleton logger.
#include "Logger.h"

#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <iostream>

Logger::Logger() {}
Logger::~Logger() { close(); }

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

bool Logger::init(const std::string& logFile, bool append) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_logFile.is_open()) m_logFile.close();
    // std::ios::trunc: the file is replaced, so it always describes one run.
    m_logFile.open(logFile, std::ios::out | (append ? std::ios::app : std::ios::trunc));
    m_path = m_logFile.is_open() ? logFile : std::string();
    return m_logFile.is_open();
}

void Logger::close() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_logFile.is_open()) {
        m_logFile.close();
    }
    m_path.clear();
}

std::string Logger::logPath() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_path;
}

void Logger::setLevel(LogLevel level) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_minLevel = level;
}

Logger::LogLevel Logger::level() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_minLevel;
}

void Logger::setConsole(bool enabled) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_console = enabled;
}

void Logger::setStepTotal(int total) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stepTotal = total > 0 ? total : 0;
}

void Logger::resetSteps() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_step = 0;
}

int Logger::currentStep() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_step;
}

void Logger::log(LogLevel level, const std::string& message) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (level < m_minLevel) return;
    const std::string line = getTimestamp() + " [" + levelToString(level) + "] " + message;
    // Diagnostics go to stderr so stdout stays clean for real data / pipes.
    if (m_console) std::cerr << line << '\n';
    if (m_logFile.is_open()) {
        m_logFile << line << '\n';
        m_logFile.flush();
    }
}

std::string Logger::vformat(const char* fmt, va_list ap) {
    va_list ap2;
    va_copy(ap2, ap);
    const int n = std::vsnprintf(nullptr, 0, fmt, ap);
    std::string msg;
    if (n > 0) {
        msg.resize((size_t)n);
        std::vsnprintf(&msg[0], (size_t)n + 1, fmt, ap2);
    }
    va_end(ap2);
    return msg;
}

void Logger::vlog(LogLevel level, const char* fmt, va_list ap) {
    // Skip the formatting cost entirely for a filtered-out level.
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (level < m_minLevel) return;
    }
    log(level, vformat(fmt, ap));
}

void Logger::vlogPrefixed(LogLevel level, const char* head, const char* fmt, va_list ap) {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (level < m_minLevel) return;
    }
    log(level, std::string(head) + vformat(fmt, ap));
}

void Logger::logf(LogLevel level, const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); vlog(level, fmt, ap); va_end(ap);
}
void Logger::debug(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); vlog(LogLevel::Debug, fmt, ap); va_end(ap);
}
void Logger::info(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); vlog(LogLevel::Info, fmt, ap); va_end(ap);
}
void Logger::warn(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); vlog(LogLevel::Warning, fmt, ap); va_end(ap);
}
void Logger::error(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); vlog(LogLevel::Error, fmt, ap); va_end(ap);
}

void Logger::step(const char* fmt, ...) {
    // The number is taken even when the level filters the line out, so the
    // surviving steps keep the numbering they would have had.
    int n, total;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        n = ++m_step;
        total = m_stepTotal;
    }
    char head[40];
    if (total > 0) std::snprintf(head, sizeof head, "==> [%d/%d] ", n, total);
    else           std::snprintf(head, sizeof head, "==> [step %d] ", n);
    va_list ap; va_start(ap, fmt);
    vlogPrefixed(LogLevel::Info, head, fmt, ap);
    va_end(ap);
}

void Logger::detail(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt);
    vlogPrefixed(LogLevel::Info, "      ", fmt, ap);
    va_end(ap);
}

void Logger::detailDebug(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt);
    vlogPrefixed(LogLevel::Debug, "      ", fmt, ap);
    va_end(ap);
}

std::string Logger::getTimestamp() {
    // Whole seconds only: these logs are read by eye, and sub-second precision
    // just adds noise. Time a stage with an explicit elapsed-time line instead.
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[32];
    std::snprintf(buf, sizeof buf, "%04d-%02d-%02d %02d:%02d:%02d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

const char* Logger::levelToString(LogLevel level) {
    switch (level) {
        case LogLevel::Debug:   return "DEBUG";
        case LogLevel::Info:    return "INFO";
        case LogLevel::Warning: return "WARN";
        case LogLevel::Error:   return "ERROR";
        default:                return "INFO";
    }
}
