#ifndef HIGHLEVELCBS_HPP
#define HIGHLEVELCBS_HPP

#include "TestInterface.hpp"
#include "HighLevelCBS.hpp"

class HighLevelCBSTests : public TestInterface{
private:
    void it_gets_a_path_that_has_no_conflicts();
    void it_can_find_a_solution_if_agents_have_same_goal();
public:
    void run(){// This is placed in the header to help ensure all tests are run
        //it_gets_a_path_that_has_no_conflicts();
        it_can_find_a_solution_if_agents_have_same_goal();
    }
};

#endif
