#include "Logger.hpp"

Logger::Logger(std::string logFile, bool clear){
    this->logFile = logFile;
    if (clear) {
        remove(&(std::string{std::filesystem::current_path()} + "/" + logFile)[0]);
    }
}

void Logger::log(std::string msg){
    // std::ofstream logFile;
    // logFile.open(std::string{std::filesystem::current_path()} + "/" + logFile, std::ofstream::app);
    // logFile << msg;
    // logFile.close();
    (*begin()) << msg; end();
}

std::ofstream* Logger::begin(){
    ostream.open(std::string{std::filesystem::current_path()} + "/" + logFile, std::ofstream::app);
    return &ostream;
}
void Logger::end(){
    ostream.close();
}