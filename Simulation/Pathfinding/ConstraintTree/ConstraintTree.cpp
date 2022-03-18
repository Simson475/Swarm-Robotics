#include "ConstraintTree.hpp"

std::shared_ptr<ConstraintTree> ConstraintTree::getParent(){}
void ConstraintTree::setChildren(){}
std::shared_ptr<ConstraintTree> ConstraintTree::getLowestCostNode(){}
Solution ConstraintTree::getSolution(){
    return solution;
}
void ConstraintTree::setSolution(Solution s){
    this->solution = s;
}
void ConstraintTree::setSolution(std::vector<Path> paths, std::vector<AgentInfo> agentInfo){
    Solution solution;
    solution.paths = paths;
    setSolution(solution);
}
std::vector<Conflict> ConstraintTree::findConflicts(){
    std::vector<Conflict> conflicts{3};
    return conflicts;
}
float ConstraintTree::getCost(){
    //Implement Lat
    return 4.20;
}

/*bool ConstraintTree::operator()(ConstraintTree a, ConstraintTree b){
    return a.cost < b.cost;
}*/