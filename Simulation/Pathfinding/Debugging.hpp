#ifndef DEBUGGING_HPP
#define DEBUGGING_HPP

class Error;

#include <fstream>
#include <iostream>
#include <cstdio>
#include <filesystem>

class Error {
public: 
    static void log(std::string err){
        std::ofstream errFile;
        errFile.open(std::string{std::filesystem::current_path()} + "/err.txt", std::ofstream::app);
        errFile << err;
        errFile.close();
    }
};

#endif