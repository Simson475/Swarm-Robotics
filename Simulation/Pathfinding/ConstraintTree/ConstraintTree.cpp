#include "ConstraintTree.hpp"

std::shared_ptr<ConstraintTree> ConstraintTree::getParent(){
    return this->parent;
}
void ConstraintTree::setChildren(std::vector<std::shared_ptr<ConstraintTree>> children){
    this->children = children;
}
std::shared_ptr<ConstraintTree> ConstraintTree::getLowestCostNode(){
    std::shared_ptr<ConstraintTree> lowestCostNode = shared_from_this();
    
    for (auto child : children){
        auto lowestFromChild = child->getLowestCostNode();
        if (lowestFromChild->getCost() < lowestCostNode->getCost()){
            lowestCostNode = lowestFromChild;
        }
    }

    return lowestCostNode;
}
Solution ConstraintTree::getSolution(){
    return solution;
}
void ConstraintTree::setSolution(Solution s){
    this->solution = s;
}
void ConstraintTree::setSolution(std::vector<Path> paths, std::vector<std::shared_ptr<Agent>> agents){
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