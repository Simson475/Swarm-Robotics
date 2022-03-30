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
    std::vector<Conflict> conflicts{};
    int size = solution.paths.size();
    for (int i = 0; i < size; ++i){
        Path p1 = solution.paths[i];
        for (int j = i; j < size; ++j){
            Path p2 = solution.paths[j];
            // Check if there is a conflict between path i (p1) and j (p2)
            int p2Index = 0;
            for (auto a1 : p1.actions){
                int numActions = solution.paths.size();
                for (int k = p2Index; k < numActions; ++k){
                    auto a2 = p2.actions[k];
                    int a1Start = a1.timestamp;
                    int a1End = a1Start + a1.duration;
                    int a2Start = a2.timestamp;
                    int a2End = a1Start + a2.duration;
                    if ((a1Start >= a2Start) && (a1Start <= a2End)){
                        std::vector<int> agents = {i, j};
                        int cStart = ((a1Start > a2Start) ? a1Start : a2Start);//max start
                        int cEnd = ((a1End < a2End) ? a1End : a2End);//min end
                        //TODO ensure that the delta is close to the actual value, but >=.
                        int delta = 1;// Small time delta aprx the time it takes a robot to move through a vertex.
                        if (!a1.isWaitAction() || !a2.isWaitAction()){
                            // There are multiple cases:
                            // Vertex conflicts:
                            // Case 1: Following too closely
                            // Case 2: Both agents are moving to the same vertex
                            // Edge conflicts:
                            // Case 3: Both agents are moving on the same edge
                            // Swap conflict:
                            // case 4: Both agents are moving on opposing edges between the same two nodes
                            bool case1 = (a1.endVertex == a2.startVertex && (a1End >= a2Start - delta))//agent i follows agent j
                                      || (a2.endVertex == a1.startVertex && (a2End >= a1Start - delta));//agent j follows agent i
                            bool case2 = a1.endVertex == a2.endVertex && (std::abs(a1End - a2End) <= delta);
                            bool case3 = a1.startVertex == a2.startVertex && a1.endVertex == a2.endVertex;
                            bool case4 = a1.startVertex == a2.endVertex && a2.startVertex == a1.endVertex;
                            if (case1){
                                // Limit the conflict time frame (it is only delta length for this case)
                                if (a1.endVertex == a2.startVertex){
                                    cStart = a2Start - delta;
                                    cEnd = a2Start;
                                }
                                else {
                                    cStart = a1Start - delta;
                                    cEnd = a1Start;
                                }
                                conflicts.push_back(Conflict(agents, cStart, cEnd,
                                    Location(ELocationType::VERTEX_LOCATION, a1.startVertex)));
                                continue;
                            }
                            if (case2){
                                cStart = ((a1End > a2End) ? a2Start : a1Start);//min of the end times
                                cEnd = cStart + delta;
                                conflicts.push_back(Conflict(agents, cStart, cEnd,
                                    Location(ELocationType::VERTEX_LOCATION, a1.startVertex)));
                                continue;
                            }
                            if (case3){
                                conflicts.push_back(Conflict(agents, cStart, cEnd,
                                    Location(ELocationType::VERTEX_LOCATION, a1.startVertex)));
                                continue;
                            }
                            if (case4){
                                conflicts.push_back(Conflict(agents, cStart, cEnd,
                                    Location(ELocationType::VERTEX_LOCATION, a1.startVertex)));
                                continue;
                            }
                        }//End of conflicts with edge actions only
                        else{
                            // There is one vertex conflict case that involves waiting:
                            // Case 1: An agent is moving to a vertex an agent is waiting on
                            bool case1 = (a1.endVertex == a2.startVertex && a2.isWaitAction())
                                      || (a2.endVertex == a1.startVertex && a1.isWaitAction());
                            if (case1){
                                conflicts.push_back(Conflict(agents, cStart, cEnd,
                                    Location(ELocationType::VERTEX_LOCATION, a1.startVertex)));
                                continue;
                            }
                        }
                    }
                }
            }
        }
    }
    return conflicts;
}
float ConstraintTree::getCost(){
    float cost = 0;
    for (Path p : this->solution.paths){
        cost += p.cost;
    }
    return cost;
}

/*bool ConstraintTree::operator()(ConstraintTree a, ConstraintTree b){
    return a.cost < b.cost;
}*/