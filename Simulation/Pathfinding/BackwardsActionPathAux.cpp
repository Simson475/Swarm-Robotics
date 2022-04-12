#include "BackwardsActionPathAux.hpp"

BackwardsActionPathAux::BackwardsActionPathAux(Action action, float heuristic, std::shared_ptr<BackwardsActionPathAux> successor){
    this->action = action;
    this->heuristic = heuristic;
    this->successor = successor;
}

BackwardsActionPathAux::BackwardsActionPathAux(const BackwardsActionPathAux &a){
    this->action = a.action;
    this->heuristic = a.heuristic;
    this->successor = a.successor;
}

Path BackwardsActionPathAux::getPath(float currentTime){
    Path path{};
    path.actions.push_back(this->action);
    path.cost = this->action.duration;
    std::shared_ptr<BackwardsActionPathAux> successor = this->successor;
    while (successor != nullptr){
        path.actions.push_back(successor->action);
        path.cost += successor->action.duration;
        successor = successor->successor;
    }
    return path;
}

void BackwardsActionPathAux::operator=(const BackwardsActionPathAux &a){
    action = a.action;
    heuristic = a.heuristic;
    successor = a.successor;
}

/* Comparator for use in priority queue (must be global or you will need a compare class) */
bool operator> (const BackwardsActionPathAux &a, const BackwardsActionPathAux &b){
    float aPriority = a.action.timestamp + a.action.duration + a.heuristic;
    float bPriority = b.action.timestamp + b.action.duration + b.heuristic;
    // Use the complete priority if they are not equal, otherwise use the heuristic
    // This avoids a lot of extra states being explores when it might not be necesary
    return aPriority == bPriority ? a.heuristic > b.heuristic : aPriority > bPriority;
}
bool operator< (const BackwardsActionPathAux &a, const BackwardsActionPathAux &b){
    float aPriority = a.action.timestamp + a.action.duration + a.heuristic;
    float bPriority = b.action.timestamp + b.action.duration + b.heuristic;
    // Use the complete priority if they are not equal, otherwise use the heuristic
    // This avoids a lot of extra states being explores when it might not be necesary
    return aPriority == bPriority ? a.heuristic < b.heuristic : aPriority < bPriority;
}

std::string BackwardsActionPathAux::toString(){
    return "{action:" + this->action.toString() + ",prio:" + std::to_string(this->action.timestamp + this->action.duration + this->heuristic) + "}";
}