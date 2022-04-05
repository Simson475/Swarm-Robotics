#include "ConstraintTree.hpp"
#include "Debugging.hpp"

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
/**
 * Creates a solution from the paths (Does not update the existing, but creates a completely new one!)
 * 
 * @param paths 
 * @param agents 
 */
void ConstraintTree::setSolution(std::vector<Path> paths, std::vector<AgentInfo> agents){
    Solution solution;
    solution.paths = paths;
    setSolution(solution);
}
std::vector<Conflict> ConstraintTree::findConflicts(){
    std::vector<Conflict> conflicts;
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
                        if (isVertexConflict(a1, a2)){
                            conflicts.push_back(getVertexConflict({i, j}, a1, a2));
                        }
                        else if (isSwapConflict(a1, a2)){ //Perhaps make the checks tighter so else isnt needed
                            conflicts.push_back(getSwapConflict(i, a1, a2));
                            conflicts.push_back(getSwapConflict(j, a2, a1));
                        }
                        else if (isEdgeConflict(a1, a2)){
                            conflicts.push_back(getEdgeConflict({i, j}, a1, a2));
                        } 
                        else if (isFollowConflict(a1, a2)){
                            conflicts.push_back(getFollowConflict({i, j}, a1, a2));
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
 * Both agents are moving on the same edge
 * @param a1 
 * @param a2 
 * @return true|false 
 */
bool ConstraintTree::isEdgeConflict(Action a1, Action a2){
    return false; // TODO remove again :)
    bool theEdgeExists = false;
    for (auto e : a1.startVertex->getEdges()){
        if (e->getEndVertex() == a2.endVertex){
            theEdgeExists = true;
            break;
        }
    }
    return theEdgeExists && a1.startVertex == a2.startVertex && a1.endVertex == a2.endVertex;
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
    // Case 1: Both agents are moving to the same vertex
    // Case 2: An agent is moving to a vertex an agent is waiting on
    // Case 3: Both agents are waiting on the same vertex (covered )
    int a1End = a1.timestamp + a1.duration;
    int a2End = a2.timestamp + a2.duration;
    bool movingToSameVertexWithinDelta = a1.endVertex == a2.endVertex && (std::abs(a1End - a2End) <= DELTA);
    bool a2MovesToa1WaitVertex = (a2.endVertex == a1.startVertex && a1.isWaitAction() && arriveWithinDelta(a1, a2));
    bool a1MovesToa2WaitVertex = (a1.endVertex == a2.startVertex && a2.isWaitAction() && arriveWithinDelta(a1, a2));
    
    // if (movingToSameVertexWithinDelta || a2MovesToa1WaitVertex || a1MovesToa2WaitVertex) {
    //     std::cout << a1.toString() << " " << a2.toString() << "\n";
    //     std::cout << movingToSameVertexWithinDelta << a2MovesToa1WaitVertex << a1MovesToa2WaitVertex << "\n";
    // }

    return movingToSameVertexWithinDelta || a2MovesToa1WaitVertex || a1MovesToa2WaitVertex;
}

/**
 * PRE: The actions are overlapping
 * 
 * @param a1 
 * @param a2 
 * @return true|false 
 */
bool ConstraintTree::isFollowConflict(Action a1, Action a2){
    return a1.endVertex == a2.startVertex // a1 ends up where a2 starts
        && (std::abs(a1.timestamp + a1.duration - a2.timestamp) <= DELTA); // a1 arrives before delta has passed
}

/**
 * PRE: The actions are overlapping
 * 
 * @param a1 
 * @param a2 
 * @return true|false 
 */
bool ConstraintTree::isSwapConflict(Action a1, Action a2){
    return a1.startVertex == a2.endVertex && a2.startVertex == a1.endVertex && a1.startVertex != a2.startVertex && actionsOverlap(a1, a2);
}

/**
 * PRE: The actions are overlapping
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
        if (e->getEndVertex() == a2.endVertex){
            edge = e;
            break;
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
    // Vertex conflicts:
    // Case 1: Both agents are moving to the same vertex
    // Case 2: An agent is moving to a vertex an agent is waiting on
    // Case 3: Both are waiting on the same vertex
    float a1Start = a1.timestamp;
    float a2Start = a2.timestamp;
    float a1End = a1.timestamp + a1.duration;
    float a2End = a2.timestamp + a2.duration;
    float cStart, cEnd;
    
    bool arriveAtSameTime = a1.endVertex == a2.endVertex && (std::abs(a1End - a2End) <= DELTA);
    bool a1w = a1.isWaitAction(), a2w = a2.isWaitAction();

    if (arriveAtSameTime){
        cStart = ((a1End > a2End) ? a2End : a1End);//min of the end times
        cEnd = cStart + DELTA;
        auto conflict = Conflict(
            conflictAgents,
            cStart, cEnd,
            Location(ELocationType::VERTEX_LOCATION, a1.endVertex));
        return conflict;
    }
    else if ((a1w && !a2w) || (!a1w && a2w)){ //case2 (an agent is moving to a vertex and agent is waiting on)
        // std::cout <<"test1";
        cStart = a1.isWaitAction() ? a2End : a1End;// arrival time of moving action
        cEnd =  (a1.isWaitAction() ? a1End : a2End) + DELTA;//max end (end of the wait action) + delta
        auto conflict = Conflict(
            conflictAgents,
            cStart, cEnd,
            Location(ELocationType::VERTEX_LOCATION, a1.endVertex));
        return conflict;
    }
    else{//case3 (both are waiting)
    // std::cout << "test2";
        cStart = std::max(a1Start, a2Start);//max start time
        cEnd = std::min(a1End, a2End) + DELTA;//min end (end of the wait action) + delta
        auto conflict = Conflict(
            conflictAgents,
            cStart, cEnd,
            Location(ELocationType::VERTEX_LOCATION, a1.endVertex));
        return conflict;
    }
}

/**
 * PRE: isVertexConflict must have returned true on the two actions
 * 
 * @param conflictAgents
 * @param a1
 * @param a2
 * @return A vertex conflict
 */
Conflict ConstraintTree::getFollowConflict(std::vector<int> conflictAgents, Action a1, Action a2){
    float a1Start = a1.timestamp;
    float a2Start = a2.timestamp;
    float a1End = a1.timestamp + a1.duration;
    float a2End = a2.timestamp + a2.duration;
    float cStart = (a1Start > a2Start) ? a1Start : a2Start;//max start time
    float cEnd = ((a1End < a2End) ? a1End : a2End) + DELTA;//min end time + delta;

    auto conflict = Conflict(
        conflictAgents,
        cStart, cEnd,
        Location(ELocationType::VERTEX_LOCATION, a2.startVertex));
    return conflict;
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
    int a2End = a2Start + a2.duration;
    int cStart = ((a1Start > a2Start) ? a1Start : a2Start);//max start
    int cEnd = ((a1End < a2End) ? a1End : a2End);//min end
    // Conflict location
    std::shared_ptr<Edge> edge;
    for (auto e : a1.startVertex->getEdges()){
        if (e->getEndVertex() == a2.startVertex){
            edge = e;
            break;
        }
    }
    return Conflict(
        conflictAgents,
        cStart,
        cEnd,
        Location(ELocationType::EDGE_LOCATION, edge)
    );
}

bool ConstraintTree::actionsOverlap(Action a1, Action a2){
    int a1Start = a1.timestamp;
    int a1End = a1Start + a1.duration;
    int a2Start = a2.timestamp;
    int a2End = a2Start + a2.duration;
    int maxStart = ((a1Start > a2Start) ? a1Start : a2Start);//max start
    int minEnd = ((a1End < a2End) ? a1End : a2End);//min end
    return maxStart <= minEnd;
}

/**
 * Returns whether one of the actions arrive within delta of where the other action ends
 * 
 * @param a1 
 * @param a2 
 * @return true|false
 */
bool ConstraintTree::arriveWithinDelta(Action a1, Action a2){
    int a1Start = a1.timestamp;
    int a1End = a1Start + a1.duration;
    int a2Start = a2.timestamp;
    int a2End = a2Start + a2.duration;
    
    return (!a1.isWaitAction() && a2Start < a1End && a1End < a2End + DELTA) || (!a2.isWaitAction() && a1Start < a2End && a2End < a1End + DELTA);
}