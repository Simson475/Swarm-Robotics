#ifndef CONFLICT_TREE_HPP
#define CONFLICT_TREE_HPP

#include <queue>
#include "Constraint.hpp"
#include "Solution.hpp"
#include "Conflict.hpp"

class ConflictTree {
  public:
    ConflictTree();// Constructor

    std::vector<Constraint> constraints;
    float cost;
    ConflictTree& parent;
    std::vector<ConflictTree> children;

    ConflictTree& getLowestCostNode();
    Solution getSolution();
    std::vector<Conflict> getConflicts();

  private:
    Solution solution;
    std::vector<Conflict> conflicts;
    Solution findSolution();
    std::vector<Conflict> findConflicts();
};

#endif