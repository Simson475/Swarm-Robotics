#ifndef TEST_INTERFACE_HPP
#define TEST_INTERFACE_HPP

#include <cassert>
#include <sys/stat.h>
#include <cmath>
#include "Logger.hpp"

class TestInterface {
    virtual void run() = 0;
};

#endif