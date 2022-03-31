#include "ActionPathAux.hpp"

ActionPathAux::ActionPathAux(Action action, int priority, std::shared_ptr<ActionPathAux> predecessor){
    this->action = action;
    this->priority = priority;
    this->predecessor = predecessor;
}

ActionPathAux::ActionPathAux(const ActionPathAux &a){
    this->action = a.action;
    this->priority = a.priority;
    this->predecessor = a.predecessor;
}

Path ActionPathAux::getPath(){
    Path path{};
    path.actions.emplace(path.actions.begin(), this->action);
    path.cost = this->action.duration;
    std::shared_ptr<ActionPathAux> predecessor = this->predecessor;
    while (predecessor != nullptr){
        path.actions.emplace(path.actions.begin(), predecessor->action);
        path.cost += predecessor->action.duration;
        predecessor = predecessor->predecessor;
    }
    return path;
}

void ActionPathAux::operator=(const ActionPathAux &a){
    action = a.action;
    priority = a.priority;
    predecessor = a.predecessor;
}

/* Comparator for use in priority queue (must be global or you will need a compare class) */
bool operator< (const ActionPathAux &a, const ActionPathAux &b){
    return a.priority > b.priority;
}