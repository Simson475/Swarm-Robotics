#ifndef HIGHLEVELCBS_HPP
#define HIGHLEVELCBS_HPP

#include "TestInterface.hpp"
#include "HighLevelCBS.hpp"

class HighLevelCBSTests : public TestInterface{
private:
    void it_gets_a_path_that_has_no_conflicts();
    void it_can_find_a_solution_if_agents_have_same_goal();
    void it_can_find_a_solution_if_agents_intersect_at_one_agents_goal();
    void it_can_find_a_solution_in_a_big_graph();
    void it_can_find_a_solution_in_a_graph_with_many_vertices();
    void bottleneck_conflicts_are_complex();
    void divided_bottlenecks_conflicts_are_complex();
    void divided_connected_bottlenecks_conflicts_are_complex();
    void it_can_find_solution_in_custom_graph();
    void it_can_find_solution_in_simulation_map();
public:
    void run(){// This is placed in the header to help ensure all tests are run
        // it_gets_a_path_that_has_no_conflicts();
        // it_can_find_a_solution_if_agents_have_same_goal(); // Infeasible
        // it_can_find_a_solution_if_agents_intersect_at_one_agents_goal();
        // it_can_find_a_solution_in_a_big_graph(); // Too hard of a problem
        // it_can_find_a_solution_in_a_graph_with_many_vertices();
        // bottleneck_conflicts_are_complex();
        // divided_bottlenecks_conflicts_are_complex();
        // divided_connected_bottlenecks_conflicts_are_complex();
        //it_can_find_solution_in_custom_graph();
        it_can_find_solution_in_simulation_map();
    }
};

#endif
