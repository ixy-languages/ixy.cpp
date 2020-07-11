#ifndef IXY_LOG_HPP
#define IXY_LOG_HPP

#include <iostream>
#include <cstring>

namespace ixy {

// logging code taken from https://stackoverflow.com/a/19452415/2072733

struct None {
};

template<typename First, typename Second>
struct Pair {
    First first;
    Second second;
};

template<typename List>
struct LogData {
    List list;
};

template<typename Begin, typename Value>
auto
operator<<(LogData<Begin> begin, const Value &value) -> LogData<Pair<Begin, const Value &>> {
    return {{begin.list, value}};
}

template<typename Begin, size_t n>
auto
operator<<(LogData<Begin> begin, const char (&value)[n]) -> LogData<Pair<Begin, const char *>> {
    return {{begin.list, value}};
}

inline void printList(std::ostream &os, None) {
}


template<typename Begin, typename Last>
void printList(std::ostream &os, const Pair<Begin, Last> &data) {
    printList(os, data.first);
    os << data.second;
}

template<typename List>
void debug(const char *file, int line, const char *function, const LogData<List> &data) {
    std::cout << "[DEBUG] " << file << ":" << line << " " << function << "(): ";
    printList(std::cout, data.list);
    std::cout << std::endl;
}

template<typename List>
void info(const char *file, int line, const char *function, const LogData<List> &data) {
    std::cout << "[INFO ] " << file << ":" << line << " " << function << "(): ";
    printList(std::cout, data.list);
    std::cout << std::endl;
}

template<typename List>
void warn(const char *file, int line, const char *function, const LogData<List> &data) {
    std::cerr << "[WARN ] " << file << ":" << line << " " << function << "(): ";
    printList(std::cerr, data.list);
    std::cerr << std::endl;
}

template<typename List>
void error(const char *file, int line, const char *function, const LogData<List> &data) {
    std::cerr << "[ERROR] " << file << ":" << line << " " << function << "(): ";
    printList(std::cerr, data.list);
    std::cerr << std::endl;
    abort();
}

#define debug(x) (debug(__FILE__,__LINE__,__func__,LogData<None>() << x))
#define info(x) (info(__FILE__,__LINE__,__func__,LogData<None>() << x))
#define warn(x) (warn(__FILE__,__LINE__,__func__,LogData<None>() << x))
#define error(x) (error(__FILE__,__LINE__,__func__,LogData<None>() << x))

}  // namespace ixy

#endif //IXY_LOG_HPP
