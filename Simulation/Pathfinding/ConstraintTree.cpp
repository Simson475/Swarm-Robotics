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
                    if (a2Start > a1End + TIME_AT_VERTEX) { break; }// Skip the remainding actions in p2 since they can't have overlaps
                    if (a1End > a2End) { p2Index++; }// Update p2Index to only start looking at the actions in p2 that can overlap with the next a1
                    int maxStart = ((a1Start > a2Start) ? a1Start : a2Start);//max start
                    int minEnd = ((a1End < a2End) ? a1End : a2End);//min end

                    // These actions overlap
                    if (maxStart <= minEnd + TIME_AT_VERTEX){
                        if (isVertexConflict(a1, a2)){
                            // std::cout << "Vertex conflict\n";
                            conflicts.push_back(getVertexConflict({i, j}, a1, a2));
                            // return conflicts;
                        }
                        else if (isSwapConflict(a1, a2)){ //Perhaps make the checks tighter so else isnt needed
                            // std::cout << "Swap conflict\n";
                            conflicts.push_back(getSwapConflict(i, a1, a2));
                            conflicts.push_back(getSwapConflict(j, a2, a1));
                            // return conflicts;
                        }
                        else if (isEdgeConflict(a1, a2)){
                            // std::cout << "Edge conflict\n";
                            conflicts.push_back(getEdgeConflict({i, j}, a1, a2));
                            // return conflicts;
                        } 
                        else if (isFollowConflict(a1, a2)){
                            // std::cout << "Follow conflict\n";
                            conflicts.push_back(getFollowConflict({i, j}, a1, a2));
                            // return conflicts;
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
    // Case 3: Both agents are waiting on the same vertex (covered by case 2)
    bool movingToSameVertexWithinDelta = a1.endVertex == a2.endVertex && (arriveWithinDelta(a1, a2) || arriveWithinDelta(a2, a1));
    bool a2MovesToa1WaitVertex = (a2.endVertex == a1.startVertex && a1.isWaitAction() && arriveWithinActionAndDelta(a2, a1));
    bool a1MovesToa2WaitVertex = (a1.endVertex == a2.startVertex && a2.isWaitAction() && arriveWithinActionAndDelta(a1, a2));

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
        && (std::abs(a1.timestamp + a1.duration - a2.timestamp) <= TIME_AT_VERTEX); // a1 arrives before delta has passed
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
        Location(edge));
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
    
    bool arriveAtSameTime = a1.endVertex == a2.endVertex && (arriveWithinDelta(a1, a2) || arriveWithinDelta(a2, a1))
        && !a1.isWaitAction() && !a2.isWaitAction();
    bool a1w = a1.isWaitAction(), a2w = a2.isWaitAction();

    if (arriveAtSameTime){
        // Error::log("Arrive at same time\n");
        // Error::log(a1.toString() + "\n");
        // Error::log(a2.toString() + "\n");
        cStart = std::max(a1End, a2End);
        cEnd = std::min(a1End, a2End) + TIME_AT_VERTEX;
        auto conflict = Conflict(
            conflictAgents,
            cStart, cEnd,
            Location(a1.endVertex));
        // Error::log(conflict.toString() + "\n");
        return conflict;
    }
    else if ((a1w && !a2w) || (!a1w && a2w)){ //case2 (an agent is moving to a vertex and agent is waiting on)
        // Error::log("One waits\n");
        // Error::log(a1.toString() + "\n");
        // Error::log(a2.toString() + "\n");
        cStart = a1.isWaitAction() ? a2End : a1End;// arrival time of moving action
        float endOfWait = (a1.isWaitAction() ? a1End : a2End) + TIME_AT_VERTEX;//end of the wait action + delta
        float endOfMove = cStart + TIME_AT_VERTEX;
        cEnd = std::min(endOfWait, endOfMove);
        auto conflict = Conflict(
            conflictAgents,
            cStart, cEnd,
            Location(a1.endVertex));
        // Error::log(conflict.toString() + "\n");
        return conflict;
    }
    else{//case3 (both are waiting)
        // Error::log("Both waits\n");
        // Error::log(a1.toString() + "\n");
        // Error::log(a2.toString() + "\n");
        cStart = std::max(a1Start, a2Start);//max start time
        cEnd = std::min(a1End, a2End) + TIME_AT_VERTEX;//min end + delta
        auto conflict = Conflict(
            conflictAgents,
            cStart, cEnd,
            Location(a1.endVertex));
        // Error::log(conflict.toString() + "\n");
        return conflict;
    }
}

/**
 * PRE: isFollowConflict must have returned true on the two actions
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
    float cStart = std::max(a1Start, a2Start);//max start time
    float cEnd = std::min(a1End, a2End) + TIME_AT_VERTEX;//min end time + delta

    auto conflict = Conflict(
        conflictAgents,
        cStart, cEnd,
        Location(a2.startVertex));
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
    float cStart = std::max(a1Start, a2Start);//max start time
    float cEnd = std::min(a1End, a2End);//min end time
    // Conflict location
    auto edge = a1.startVertex->getEdge(a1.endVertex);
    return Conflict(
        conflictAgents,
        cStart,
        cEnd,
        Location(edge)
    );
}

bool ConstraintTree::actionsOverlap(Action a1, Action a2){
    int a1Start = a1.timestamp;
    int a1End = a1Start + a1.duration;
    int a2Start = a2.timestamp;
    int a2End = a2Start + a2.duration;
    int maxStart = std::max(a1Start, a2Start);//max start
    int minEnd = std::min(a1End, a2End);//min end
    return maxStart <= minEnd;
}

/**
 * Returns whether a1 arrive within delta of where a2 ends
 * 
 * @param a1 
 * @param a2 
 * @return true|false
 */
bool ConstraintTree::arriveWithinDelta(Action a1, Action a2){
    float a1Start = a1.timestamp;
    float a1End = a1Start + a1.duration;
    float a2Start = a2.timestamp;
    float a2End = a2Start + a2.duration;
    
    return (a1End + TIME_AT_VERTEX >= a2End && a1End <= a2End + TIME_AT_VERTEX);
}

/**
 * Returns whether a1 arrive within delta of where a2 ends
 * 
 * @param a1 
 * @param a2 
 * @return true|false
 */
bool ConstraintTree::arriveWithinActionAndDelta(Action a1, Action a2){
    int a1Start = a1.timestamp;
    int a1End = a1Start + a1.duration;
    int a2Start = a2.timestamp;
    int a2End = a2Start + a2.duration;
    
    return (a1End >= a2Start && a1End <= a2End + TIME_AT_VERTEX);
}

std::vector<Constraint> ConstraintTree::getConstraints(int agentId){
    std::vector<Constraint> agentConstraints;
    for (auto c : this->constraints){
        if (c.agentId == agentId){
            agentConstraints.push_back(c);
        }
    }
    return agentConstraints;
}
std::vector<Constraint> ConstraintTree::getConstraints(){
    return this->constraints;
}
void ConstraintTree::setConstraints(std::vector<Constraint> constraints){
    this->constraints = constraints;
}
void ConstraintTree::addConstraint(Constraint constraint){
    // Merge the constraint with existing constraints
    //TODO can this be done more efficient?
    bool addedConstraint = false;
    auto iterator = this->constraints.begin();
    for (auto c : this->constraints){
        if (c.location == constraint.location
            && c.agentId == constraint.agentId
        ){
            float maxStart = std::max(constraint.timeStart, c.timeStart);
            float minEnd = std::min(constraint.timeEnd, c.timeEnd);
            if (maxStart <= minEnd){
                float minStart = std::min(constraint.timeStart, c.timeStart);
                float maxEnd = std::max(constraint.timeEnd, c.timeEnd);
                this->constraints.erase(iterator);
                this->constraints.push_back(Constraint(c.agentId, c.location, minStart, maxEnd));
                addedConstraint = true;
                break;
            }
        }
        iterator++;
    }
    if ( ! addedConstraint){
        this->constraints.push_back(constraint);
    }
}