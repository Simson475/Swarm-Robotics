#include "AgentInfo.hpp"

AgentInfo::AgentInfo(){
    
}
AgentInfo::AgentInfo(const AgentInfo& a){
    id = a.id;
    currentAction = a.currentAction;
    goal = a.goal;
    _isWorking = a._isWorking;
}
AgentInfo::AgentInfo(AgentInfo* a){
    id = a->getId();
    currentAction = a->getCurrentAction();
    goal = a->getGoal();
    _isWorking = a->isWorking();
}
AgentInfo::AgentInfo(AgentInfo&& a){
    id = a.getId();
    currentAction = a.getCurrentAction();
    goal = a.getGoal();
    _isWorking = a.isWorking();
}
AgentInfo::AgentInfo(int id, Action currentAction, std::shared_ptr<Vertex> destination, bool isWorking){
    this->id = id;
    this->currentAction = currentAction;
    this->goal = destination;
    this->_isWorking = isWorking;
}


int AgentInfo::getId() const { return id; }
Action AgentInfo::getCurrentAction() const { return currentAction; }
std::shared_ptr<Vertex> AgentInfo::getGoal() const { return goal; }

void AgentInfo::operator=(const AgentInfo& other){
    id = other.id;
    currentAction = other.currentAction;
    goal = other.goal;
}

bool AgentInfo::isWorking(){
    return this->_isWorking;
}