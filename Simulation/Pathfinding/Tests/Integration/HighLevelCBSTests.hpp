#ifndef HIGHLEVELCBS_HPP
#define HIGHLEVELCBS_HPP

#include "TestInterface.hpp"
#include "HighLevelCBS.hpp"

class HighLevelCBSTests : public TestInterface{
private:
    void it_gets_a_path_that_has_no_conflicts();
public:
    void run(){// This is placed in the header to help ensure all tests are run
        it_gets_a_path_that_has_no_conflicts();
    }
};

#endif
