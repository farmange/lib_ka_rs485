#include "logger.h"

#include <iostream>
#include <sstream>
#include <iomanip>

namespace KinovaApi
{
    void Logger::print(logging_level_t level, std::string s)
    {
        std::string level_string;
        if (level < level_)
        {
            return;
        }
        switch (level)
        {
        case DEBUG:
            level_string = "DEBUG";
            break;
        case INFO:
            level_string = " INFO";
            break;
        case WARNING:
            level_string = " WARN";
            break;
        case ERROR:
            level_string = "ERROR";
            break;
        }
        std::cout << "[" << level_string << "] : " << s;
        std::cout.flush();
        return;
    }
    void Logger::info_stream(std::stringstream &ss)
    {
        print(INFO, ss.str() + '\n');
    }
    void Logger::info(std::string s)
    {
        print(INFO, s + '\n');
    }
    void Logger::debug_stream(std::stringstream &ss)
    {
        print(DEBUG, ss.str() + '\n');
    }
    void Logger::debug(std::string s)
    {
        print(DEBUG, s + '\n');
    }
    void Logger::error_stream(std::stringstream &ss)
    {
        print(ERROR, ss.str() + '\n');
    }
    void Logger::error(std::string s)
    {
        print(ERROR, s + '\n');
    }
    void Logger::warn_stream(std::stringstream &ss)
    {
        print(WARNING, ss.str() + '\n');
    }
    void Logger::warn(std::string s)
    {
        print(WARNING, s + '\n');
    }

    void Logger::hexDump(logging_level_t level, const char *desc, const void *addr, const int len)
    {
        int i;
        unsigned char buff[17];
        const unsigned char *pc = (const unsigned char *)addr;

        // Output description if given.
        if (desc != NULL)
        {
            print(level, std::string(desc) + ":\n");
        }
        // Length checks.
        if (len == 0)
        {
            print(level, "Zero length");
            return;
        }
        else if (len < 0)
        {
            print(level, "Negative length: " + len);
            return;
        }

        // std::stringstream ss;
        std::string hexdump_str = "";
        std::stringstream ss;

        // Process every byte in the data.
        for (i = 0; i < len; i++)
        {
            // Multiple of 16 means new line (with line offset).
            if ((i % 16) == 0)
            {
                // Don't print ASCII buffer for the "zeroth" line.
                if (i != 0)
                {
                    ss << "  | " << buff << std::endl;
                    print(level, ss.str());
                    ss.str("");
                }
                // Output the offset.
                ss << std::uppercase << std::setfill('0') << std::setw(4) << std::hex << i << " | ";
            }
            else if ((i % 8) == 0)
            {
                ss << "  ";
            }
            // Now the hex code for the specific character.
            ss << " " << std::setfill('0') << std::setw(2) << std::hex << (int)pc[i];

            // And buffer a printable ASCII character for later.
            if ((pc[i] < 0x20) || (pc[i] > 0x7e)) // isprint() may be better.
                buff[i % 16] = '.';
            else
                buff[i % 16] = pc[i];
            buff[(i % 16) + 1] = '\0';
        }

        while ((i % 16) != 0)
        {
            ss << "   ";
            i++;
        }

        // And print the final ASCII buffer.
        ss << "    | " << buff;
        print(level, ss.str() + ":\n");
    }

    void Logger::setLevel(logging_level_t level)
    {
        level_ = level;
    }

} // namespace KinovaApi