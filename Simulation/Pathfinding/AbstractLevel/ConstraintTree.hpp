#ifndef CONSTRAINT_TREE_HPP
#define CONSTRAINT_TREE_HPP
class ConstraintTree;

#include <vector>
#include "Constraint.hpp"
#include "Solution.hpp"
#include "Conflict.hpp"

class ConstraintTree {
public:
    ConstraintTree() = default;// Constructor
    std::vector<Conflict> getConflicts();
    Solution getSolution();
    float getSICCost();

private:
    std::vector<Constraint> constraints;
    float cost;
    ConstraintTree* parent;
    std::vector<ConstraintTree*> children;
    ConstraintTree* getLowestCostNode();
    Solution* solution;
    std::vector<Conflict> conflicts;
    bool operator() (ConstraintTree* a, ConstraintTree* b);//Comparison function for priority queue

  private:
};

#endif