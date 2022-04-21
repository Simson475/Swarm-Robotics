#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <fstream>
#include <iostream>
#include <cstdio>
#include <filesystem>
#include <memory>

class Logger {
public:
    // Singleton
    static Logger &get_instance() {
        static Logger instance;
        return instance;
    }
    Logger() = default;
    Logger(const Logger&) = delete;
    static bool enabled;
    void setLogFile(std::string);
    void log(std::string msg);
    std::ofstream* begin();
    void end();
private:
    std::string logFile;
    std::ofstream ostream;
};


#endif