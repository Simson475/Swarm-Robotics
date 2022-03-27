#ifndef ACTION_PATH_AUX_TESTS_HPP
#define ACTION_PATH_AUX_TESTS_HPP

#include "TestInterface.hpp"
#include <queue>
#include <memory>
#include "ActionPathAux.hpp"
#include "Vertex.hpp"
#include "Action.hpp"

class ActionPathAuxTests : public TestInterface {
private:
    void it_can_compare_in_priority_queue();
public:
    // Writing this in header so it is much easier to see that all functions are called
    void run(){
        it_can_compare_in_priority_queue();
    }
};

#endif