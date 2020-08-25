#ifndef IXY_LOG_HPP
#define IXY_LOG_HPP

// TODO: replace with <format>
#include <fmt/core.h>

namespace ixy {

#define debug(...) { \
    fmt::print("[DEBUG] {}:{} {}(): ", __FILE__, __LINE__, __func__); \
    fmt::print(__VA_ARGS__);                                          \
    fmt::print("\n");                                                 \
}

#define info(...) { \
    fmt::print("[INFO ] {}:{} {}(): ", __FILE__, __LINE__, __func__); \
    fmt::print(__VA_ARGS__);                                          \
    fmt::print("\n");                                                 \
}

#define warn(...) { \
    fmt::print("[WARN ] {}:{} {}(): ", __FILE__, __LINE__, __func__); \
    fmt::print(__VA_ARGS__);                                          \
    fmt::print("\n");                                                 \
}

#define error(...) { \
    fmt::print(stderr, "[ERROR] {}:{} {}(): ", __FILE__, __LINE__, __func__); \
    fmt::print(stderr, __VA_ARGS__);                                          \
    fmt::print(stderr, "\n");                                                 \
    abort();                                                                  \
}

}  // namespace ixy

#endif //IXY_LOG_HPP
