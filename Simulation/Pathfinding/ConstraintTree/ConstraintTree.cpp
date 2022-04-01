#include "ConstraintTree.hpp"

const float delta = 20; // Small time delta aprx the time it takes a robot to move through a vertex.

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
void ConstraintTree::setSolution(std::vector<Path> paths, std::vector<AgentInfo> agents){
    Solution solution;
    solution.paths = paths;//TODO this does not work??!
    setSolution(solution);
}
std::vector<Conflict> ConstraintTree::findConflicts(){
    std::vector<Conflict> conflicts{};
    int size = solution.paths.size();
    for (int i = 0; i < size; ++i){
        Path p1 = solution.paths[i];
        for (int j = i+1; j < size; ++j){
            Path p2 = solution.paths[j];
            // Check if there is a conflict between path i (p1) and j (p2)
            int p2Index = 0;
            for (auto a1 : p1.actions){
                int numActions = p2.actions.size();
                for (int k = p2Index; k < numActions; ++k){
                    auto a2 = p2.actions[k];
                    int a1Start = a1.timestamp;
                    int a1End = a1Start + a1.duration;
                    int a2Start = a2.timestamp;
                    int a2End = a1Start + a2.duration;
                    if (a2Start > a1End) { break; }// Skip the remainding actions in p2 since they can't have overlaps
                    if (a1End > a2End) { p2Index++; }// Update p2Index to only start looking at the actions in p2 that can overlap with the next a1
                    int maxStart = ((a1Start > a2Start) ? a1Start : a2Start);//max start
                    int minEnd = ((a1End < a2End) ? a1End : a2End);//min end

                    // These actions overlap
                    if (maxStart <= minEnd){
                        if (isEdgeConflict(a1, a2)){
                            std::cout << "Edge conflict: " << a1.toString() << " " << a2.toString() << "\n";
                            conflicts.push_back(getEdgeConflict({i, j}, a1, a2));
                        }
                        else if (isSwapConflict(a1, a2)){ //Perhaps make the checks tighter so else isnt needed
                            std::cout << "Swap conflict: " << a1.toString() << " " << a2.toString() << "\n";
                            conflicts.push_back(getSwapConflict(i, a1, a2));
                            conflicts.push_back(getSwapConflict(j, a2, a1));
                        }
                        else if (isVertexConflict(a1, a2)){
                            std::cout << "Vertex conflict: " << a1.toString() << " " << a2.toString() << "\n";
                            conflicts.push_back(getVertexConflict({i, j}, a1, a2));
                            std::cout << "conflict: " << conflicts[conflicts.size()-1].toString() << "\n";
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

bool ConstraintTree::operator() (std::shared_ptr<ConstraintTree> a, std::shared_ptr<ConstraintTree> b){
    return a->getCost() > b->getCost();
}

/**
 * PRE: The actions are overlapping
 * 
 * @param a1 
 * @param a2 
 * @return true|false 
 */
bool ConstraintTree::isEdgeConflict(Action a1, Action a2){
    // Edge conflicts:
    // Both agents are moving on the same edge
    return a1.startVertex == a2.startVertex && a1.endVertex == a2.endVertex;
}

/**
 * PRE: The actions are overlapping
 * 
 * @param a1 
 * @param a2 
 * @return true|false 
 */
bool ConstraintTree::isVertexConflict(Action a1, Action a2){
    // Vertex conflicts:
    // Case 1: Following too closely
    // Case 2: Both agents are moving to the same vertex
    // Case 3: An agent is moving to a vertex an agent is waiting on
    float a1Start = a1.timestamp;
    float a1End = a1.timestamp + a1.duration;
    float a2Start = a2.timestamp;
    float a2End = a2.timestamp + a2.duration;
    
    bool case1 = (a1.endVertex == a2.startVertex && a2.endVertex != a1.startVertex && (a1End - a2Start >= delta))//agent i follows agent j
              || (a2.endVertex == a1.startVertex && a1.endVertex != a2.startVertex && (a2End - a1Start >= delta));//agent j follows agent i
    bool case2 = a1.endVertex == a2.endVertex && (std::abs(a1End - a2End) <= delta);
    bool case3 = (a1.endVertex == a2.startVertex && a2.isWaitAction())
                || (a2.endVertex == a1.startVertex && a1.isWaitAction());
    if (case1 || case2 || case3)
        std::cout << std::to_string(case1) << std::to_string(case2) << std::to_string(case3) << "\n";
    return case1 || case2 || case3;
}

/**
 * PRE: The actions are overlapping
 * 
 * @param a1 
 * @param a2 
 * @return true|false 
 */
bool ConstraintTree::isSwapConflict(Action a1, Action a2){
    // Swap conflict:
    // Both agents are moving on opposing edges between the same two nodes
    return a1.startVertex == a2.endVertex && a2.startVertex == a1.endVertex;
}

/**
 * PRE: isEdgeConflict must have returned true on the two actions
 * 
 * @param conflictAgents
 * @param a1
 * @param a2
 * @return An edge conflict
 */
Conflict ConstraintTree::getEdgeConflict(std::vector<int> conflictAgents, Action a1, Action a2){
    int a1Start = a1.timestamp;
    int a1End = a1Start + a1.duration;
    int a2Start = a2.timestamp;
    int a2End = a1Start + a2.duration;
    int cStart = ((a1Start > a2Start) ? a1Start : a2Start);//max start
    int cEnd = ((a1End < a2End) ? a1End : a2End);//min end
    std::shared_ptr<Edge> edge;
    for (auto e : a1.startVertex->getEdges()){
        if (e->getEndVertex() == a2.startVertex){
            edge = e;
        }
    }
    return Conflict(conflictAgents, cStart, cEnd,
        Location(ELocationType::EDGE_LOCATION, edge));
}

/**
 * PRE: isVertexConflict must have returned true on the two actions
 * 
 * @param conflictAgents
 * @param a1
 * @param a2
 * @return A vertex conflict
 */
Conflict ConstraintTree::getVertexConflict(std::vector<int> conflictAgents, Action a1, Action a2){
    float cStart, cEnd;
    // int a1Start = a1.timestamp;
    // int a1End = a1Start + a1.duration;
    // int a2Start = a2.timestamp;
    // int a2End = a1Start + a2.duration;

    float a1Start = a1.timestamp;
    float a1End = a1.timestamp + a1.duration;
    float a2Start = a2.timestamp;
    float a2End = a2.timestamp + a2.duration;
    /**
     * There are a few vertex conflict cases:
     * Case 1: Following too closely
     * Case 2: Both agents are moving to the same vertex
     * Case 3: Both agents want to wait on the same vertex
     */
    bool case1 = (a1.endVertex == a2.startVertex && a2.endVertex != a1.startVertex && (a1End - a2Start >= delta))//agent i follows agent j
              || (a2.endVertex == a1.startVertex && a1.endVertex != a2.startVertex && (a2End - a1Start >= delta));//agent j follows agent i
    bool case2 = a1.endVertex == a2.endVertex && (std::abs(a1End - a2End) <= delta);
    // No need to check case 3, since if it is not 1 or 2 it must be 3

    std::cout << "case1: " << case1 << " case2: " << case2;
    if (case1){
        // Limit the conflict time frame (it is only delta length for this case)
        if (a1.endVertex == a2.startVertex){
            std::cout << "entered the if \n";
            cStart = a2Start;
            cEnd = a2Start + delta;
        }
        else {
            std::cout << "entered the else \n";
            cStart = a1Start;
            cEnd = a1Start + delta;
        }
        auto conflict = Conflict(
            conflictAgents,
            cStart, cEnd,
            Location(ELocationType::VERTEX_LOCATION, a1.startVertex));
        std::cout << conflict.getLocation().toString();
        return conflict;
    }
    else if (case2){
        std::cout << "Are we in case 2? \n";
        cStart = ((a1End > a2End) ? a2End : a1End);//min of the end times
        cEnd = cStart + delta;
        auto conflict = Conflict(
            conflictAgents,
            cStart, cEnd,
            Location(ELocationType::VERTEX_LOCATION, a1.endVertex));
        std::cout << conflict.getLocation().toString();
        return conflict;
    }
    else{ //case3
        std::cout << "Are we secretly waiting for lyfe? \n";
        cStart = ((a1Start > a2Start) ? a1Start : a2Start);//max start
        cEnd = ((a1End < a2End) ? a1End : a2End);//min end
        auto conflict = Conflict(
            conflictAgents,
            cStart, cEnd,
            Location(ELocationType::VERTEX_LOCATION, a1.endVertex));
        std::cout << conflict.getLocation().toString();
        return conflict;
    }
}

/**
 * PRE: isSwapConflict must have returned true on the two actions
 * 
 * @param conflictAgents
 * @param a1
 * @param a2
 * @return A swap conflict
 */
Conflict ConstraintTree::getSwapConflict(int conflictAgent, Action a1, Action a2){
    // Conflict agents
    std::vector<int> conflictAgents = { conflictAgent };
    // Conflict timespan
    int a1Start = a1.timestamp;
    int a1End = a1Start + a1.duration;
    int a2Start = a2.timestamp;
    int a2End = a1Start + a2.duration;
    int cStart = ((a1Start > a2Start) ? a1Start : a2Start);//max start
    int cEnd = ((a1End < a2End) ? a1End : a2End);//min end
    // Conflict location
    std::shared_ptr<Edge> edge;
    for (auto e : a1.startVertex->getEdges()){
        if (e->getEndVertex() == a2.startVertex){
            edge = e;
        }
    }
    return Conflict(
        conflictAgents,
        cStart,
        cEnd,
        Location(ELocationType::EDGE_LOCATION, edge)
    );
}
