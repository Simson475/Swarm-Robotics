#include "ActionPathAux.hpp"

ActionPathAux::ActionPathAux(Action action, float heuristic, std::shared_ptr<ActionPathAux> predecessor, bool hasWorked){
    this->action = action;
    this->heuristic = heuristic;
    this->predecessor = predecessor;
    this->hasWorked = hasWorked;
}

ActionPathAux::ActionPathAux(const ActionPathAux &a){
    this->action = a.action;
    this->heuristic = a.heuristic;
    this->predecessor = a.predecessor;
    this->hasWorked = a.hasWorked;
}

Path ActionPathAux::getPath() const{
    Path path{};

    path.actions.emplace(path.actions.begin(), this->action);
    path.cost = this->action.duration;
    
    std::shared_ptr<ActionPathAux> predecessor = this->predecessor;
    while (predecessor != nullptr){
        path.actions.emplace(path.actions.begin(), predecessor->action);
        path.cost += predecessor->action.duration;
        predecessor = predecessor->predecessor;
    }
    
    // Adding timestamp, so the cost is the complete cost it took to reach this from start
    path.cost += path.actions.front().timestamp;
    
    return path;
}

/**
 * PRE: This action path aux contains a working at goal action
 * 
 * @return Path 
 */
Path ActionPathAux::getPathWithoutCAT() const{
    if (this->predecessor != nullptr && this->predecessor->hasWorked == false){
        return this->getPath();
    }
    std::shared_ptr<ActionPathAux> predecessor = this->predecessor;
    while (predecessor->predecessor != nullptr && this->predecessor->hasWorked != false){
        predecessor = predecessor->predecessor;
    }
    assert(predecessor != nullptr);
    return predecessor->getPath();
}

void ActionPathAux::operator=(const ActionPathAux &a){
    action = a.action;
    heuristic = a.heuristic;
    predecessor = a.predecessor;
    hasWorked = a.hasWorked;
}

/* Comparator for use in priority queue (must be global or you will need a compare class) */
bool operator> (const ActionPathAux &a, const ActionPathAux &b){
    float aPriority = a.action.timestamp + a.action.duration + a.heuristic;
    float bPriority = b.action.timestamp + b.action.duration + b.heuristic;
    // Use the complete priority if they are not equal, otherwise use the heuristic
    // If the heuristics are equal, compare number of actions in the path
    // This avoids a lot of extra states being explores when it might not be necesary
    if(aPriority != bPriority) return aPriority > bPriority;
    if(a.heuristic != b.heuristic) return a.heuristic > b.heuristic;
    return a.getPathWithoutCAT().actions.size() > b.getPathWithoutCAT().actions.size();

}

bool operator< (const ActionPathAux &a, const ActionPathAux &b){
    float aPriority = a.action.timestamp + a.action.duration + a.heuristic;
    float bPriority = b.action.timestamp + b.action.duration + b.heuristic;
    // Use the complete priority if they are not equal, otherwise use the heuristic
    // If the heuristics are equal, compare number of actions in the path
    // This avoids a lot of extra states being explores when it might not be necesary
    if(aPriority != bPriority) return aPriority < bPriority;
    if(a.heuristic != b.heuristic) return  a.heuristic < b.heuristic;
    return a.getPathWithoutCAT().actions.size() < b.getPathWithoutCAT().actions.size();

}

std::string ActionPathAux::toString(){
    return "{action:" + this->action.toString() + ",prio:" + std::to_string(this->action.timestamp + this->action.duration + this->heuristic) + "}";
}