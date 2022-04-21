#ifndef CONSTRAINTTREE_HPP
#define CONSTRAINTTREE_HPP

#include "TestInterface.hpp"
#include "ConstraintTree.hpp"

class ConstraintTreeTests : public TestInterface{
private:
    void it_can_find_vertex_conflicts_moving_to_same_vertex();
    void it_can_find_vertex_conflicts_waiting_on_same_vertex();
    void it_can_find_vertex_conflicts_moves_to_vertex_occupied_by_wait();
    void it_does_not_find_vertex_conflict();
    void it_can_find_edge_conflicts();
    void it_can_find_swap_conflicts();
    void it_gets_sorted_in_priority_queue();
    void it_is_vertex_conflict();
    void it_is_edge_conflict();
    void it_is_swap_conflict();
    void it_is_follow_conflict();
    void it_can_get_vertex_conflict();
    void it_can_get_edge_conflict();
    void it_can_get_swap_conflict();
    void it_can_get_follow_conflict();
public:
    void run(){// This is placed in the header to help ensure all tests are run
        it_can_find_vertex_conflicts_moving_to_same_vertex();
        it_can_find_vertex_conflicts_waiting_on_same_vertex();
        it_can_find_vertex_conflicts_moves_to_vertex_occupied_by_wait();
        it_does_not_find_vertex_conflict();
        //it_can_find_edge_conflicts();
        it_can_find_swap_conflicts();
        it_gets_sorted_in_priority_queue();
        it_is_vertex_conflict();
        //it_is_edge_conflict();
        it_is_swap_conflict();
        it_is_follow_conflict();
        it_can_get_vertex_conflict();
        //it_can_get_edge_conflict();
        it_can_get_swap_conflict();
        it_can_get_follow_conflict();
    }
};

#endif
