#ifndef CONSTRAINT_TREE_HPP
#define CONSTRAINT_TREE_HPP
class ConstraintTree;

#include <vector>
#include <memory>
#include "Constraint.hpp"
#include "Solution.hpp"
#include "Conflict.hpp"

class ConstraintTree {
public:
    std::vector<std::shared_ptr<Constraint>> constraints;
    std::shared_ptr<ConstraintTree> getParent();
    void setChildren();
    std::shared_ptr<ConstraintTree> getLowestCostNode();
    Solution getSolution();
    void setSolution(Solution);
    void setSolution(std::vector<Path>, std::vector<AgentInfo>);
    std::vector<Conflict> findConflicts();
    float getCost();
private:
    std::shared_ptr<ConstraintTree> parent;
    std::vector<std::shared_ptr<ConstraintTree>> children;
    Solution solution;
    std::vector<std::shared_ptr<Conflict>> conflicts;
    bool operator() (ConstraintTree* a, ConstraintTree* b);//Comparison function for priority queue

private:
};

#endif