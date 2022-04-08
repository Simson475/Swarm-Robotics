#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <fstream>
#include <iostream>
#include <cstdio>
#include <filesystem>

class Logger {
public:
    Logger(std::string logFile, bool clear = true);
    void log(std::string msg);
    std::ofstream* begin();
    void end();
    static std::string experimentPrefix;
private:
    std::string logFile;
    std::ofstream ostream;
};


#endif