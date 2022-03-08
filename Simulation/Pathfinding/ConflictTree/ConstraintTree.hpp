#ifndef CONSTRAINT_TREE_HPP
#define CONSTRAINT_TREE_HPP

#include <queue>
#include "Constraint.hpp"
#include "Solution.hpp"
#include "Conflict.hpp"

class ConstraintTree {
  public:
    ConstraintTree();// Constructor

    std::vector<Constraint> constraints;
    float cost;
    ConstraintTree& parent;
    std::vector<ConstraintTree> children;
    ConstraintTree& getLowestCostNode();
    Solution solution;
    std::vector<Conflict> conflicts;
    bool operator() (ConstraintTree a, ConstraintTree b);//Comparison function for priority queue

  private:
};

#endif