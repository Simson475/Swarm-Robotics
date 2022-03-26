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
    path.actions.emplace(path.actions.begin(), this->predecessor->action);
    path.cost = predecessor->action.duration;
    std::shared_ptr<ActionPathAux> predecessor = this->predecessor;
    while (predecessor != nullptr){
        path.actions.emplace(path.actions.begin(), predecessor->action);
        path.cost += predecessor->action.duration;
        predecessor = predecessor->predecessor;
    }
    return path;
}

bool operator< (const ActionPathAux &a, const ActionPathAux &b){
    return a.priority > b.priority;
}