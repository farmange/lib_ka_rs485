#ifndef _LOGGER_H
#define _LOGGER_H

// C++ library headers
#include <string>
#include <sstream>
#include <iostream>

#define LOG_INFO(S_) Logger ::instance().info(S_)
#define LOG_INFO_STREAM(S_)       \
    {                             \
        std::stringstream __ss__; \
        __ss__ << S_;             \
        Logger::instance()        \
            .info_stream(__ss__); \
    }

#define LOG_DEBUG(S_) Logger ::instance().debug(S_)
#define LOG_DEBUG_STREAM(S_)       \
    {                              \
        std::stringstream __ss__;  \
        __ss__ << S_;              \
        Logger::instance()         \
            .debug_stream(__ss__); \
    }

#define LOG_ERROR(S_) Logger ::instance().error(S_)
#define LOG_ERROR_STREAM(S_)       \
    {                              \
        std::stringstream __ss__;  \
        __ss__ << S_;              \
        Logger::instance()         \
            .error_stream(__ss__); \
    }

#define LOG_WARN(S_) Logger ::instance().warn(S_)
#define LOG_WARN_STREAM(S_)       \
    {                             \
        std::stringstream __ss__; \
        __ss__ << S_;             \
        Logger::instance()        \
            .warn_stream(__ss__); \
    }

#define LOG_HEXDUMP(LEVEL_, DESC_, ADDR_, LEN_) Logger ::instance().hexDump(LEVEL_, DESC_, ADDR_, LEN_)

namespace KinovaApi
{
    class Logger
    {
        Logger(){};

    public:
        typedef enum logging_level_e
        {
            DEBUG = 0,
            INFO,
            WARNING,
            ERROR
        } logging_level_t;

        void print(logging_level_t level, std::string s);
        void info_stream(std::stringstream &ss);
        void info(std::string s);
        void debug_stream(std::stringstream &ss);
        void debug(std::string s);
        void error_stream(std::stringstream &ss);
        void error(std::string s);
        void warn_stream(std::stringstream &ss);
        void warn(std::string s);
        void hexDump(logging_level_t level, const char *desc, const void *addr, const int len);

        static Logger &instance()
        {
            static Logger myinstance;
            return myinstance;
        }

        void setLevel(logging_level_t level);

    private:
        logging_level_t level_;
    };

    // class LoggerSingleton
    // {
    //     LoggerSingleton()
    //     {
    //         //This is threadsafe in gcc, no mutex required
    //     }

    // public:
    //     static Logger &instance()
    //     {
    //         static Logger myinstance;
    //         return myinstance;
    //     }
    // };

} // namespace KinovaApi
#endif