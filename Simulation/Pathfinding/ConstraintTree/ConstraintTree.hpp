#ifndef CONSTRAINT_TREE_HPP
#define CONSTRAINT_TREE_HPP
#define DELTA 20
class ConstraintTree;

#include <vector>
#include <memory>
#include "Constraint.hpp"
#include "Solution.hpp"
#include "Conflict.hpp"
#include "AgentInfo.hpp"


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
    void setSolution(std::vector<Path>, std::vector<AgentInfo>);
    std::vector<Conflict> findConflicts();
    float getCost();
    bool operator() (std::shared_ptr<ConstraintTree> a, std::shared_ptr<ConstraintTree> b);//Comparison function for priority queue
    bool isEdgeConflict(Action a1, Action a2);
    bool isVertexConflict(Action a1, Action a2);
    bool isFollowConflict(Action a1, Action a2);
    bool isSwapConflict(Action a1, Action a2);
    Conflict getEdgeConflict(std::vector<int> conflictAgents, Action a1, Action a2);
    Conflict getVertexConflict(std::vector<int> conflictAgents, Action a1, Action a2);
    Conflict getFollowConflict(std::vector<int> conflictAgents, Action a1, Action a2);
    Conflict getSwapConflict(int conflictAgents, Action a1, Action a2);

private:
    std::shared_ptr<ConstraintTree> parent;
    std::vector<std::shared_ptr<ConstraintTree>> children;
    Solution solution;
    std::vector<std::shared_ptr<Conflict>> conflicts;
    bool actionsOverlap(Action a1, Action a2);
    bool arriveWithinDelta(Action a1, Action a2);
};

#endif