#include "Logger.hpp"

bool Logger::enabled = false;

Logger::Logger(const Logger& other){
    this->logFile = other.logFile;
}

void Logger::setLogFile(std::string logFile){
    this->logFile = logFile;
}

void Logger::log(std::string msg){
    (*begin()) << msg; end();
}

std::ofstream* Logger::begin(){
    ostream.open(std::string{std::filesystem::current_path()} + "/" + logFile, std::ofstream::app);
    return &ostream;
}
void Logger::end(){
    ostream.close();
}