#ifndef CONSTRAINT_TREE_HPP
#define CONSTRAINT_TREE_HPP
class ConstraintTree;

#include <vector>
#include <memory>
#include "Constraint.hpp"
#include "Solution.hpp"
#include "Conflict.hpp"
#include "Agent.hpp"

class ConstraintTree : public std::enable_shared_from_this<ConstraintTree> {
public:
    /* Tree elements */
    std::shared_ptr<ConstraintTree> getParent();
    void setChildren(std::vector<std::shared_ptr<ConstraintTree>> children);
    std::shared_ptr<ConstraintTree> getLowestCostNode();
    std::vector<Constraint> constraints;

    /* More CBS specific */
    Solution getSolution();
    void setSolution(Solution);
    void setSolution(std::vector<Path>, std::vector<std::shared_ptr<Agent>>);
    std::vector<Conflict> findConflicts();
    float getCost();
private:
    std::shared_ptr<ConstraintTree> parent;
    std::vector<std::shared_ptr<ConstraintTree>> children;
    Solution solution;
    std::vector<std::shared_ptr<Conflict>> conflicts;
    bool operator() (ConstraintTree* a, ConstraintTree* b);//Comparison function for priority queue
};

#endif