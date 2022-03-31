#ifndef LOWLEVEL_CBS_TESTS_HPP
#define LOWLEVEL_CBS_TESTS_HPP

#include "TestInterface.hpp"
#include "LowLevelCBS.hpp"

class LowLevelCBSTests : public TestInterface {
private:
    void pathfinding_is_correct();
public:
    void run(){// In header to make it easier to ensure every test is run
        pathfinding_is_correct();
    }
    
};

#endif