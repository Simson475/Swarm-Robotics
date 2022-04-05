#ifndef GRAPH_TESTS_HPP
#define GRAPH_TESTS_HPP

#include "TestInterface.hpp"
#include "Graph.hpp"

class GraphTests : public TestInterface {
private:
    void heuristic_costs_are_correct();
public:
    void run(){// In header so it is easier to see if all tests are run
        heuristic_costs_are_correct();
    }
};

#endif