#include "AgentInfo.hpp"

AgentInfo::AgentInfo(){
    
}
AgentInfo::AgentInfo(const AgentInfo& a){
    id = a.id;
    currentAction = a.currentAction;
    goal = a.goal;
    _isWorking = a._isWorking;
    _shouldWorkAtGoal = a._shouldWorkAtGoal;
}
AgentInfo::AgentInfo(AgentInfo* a){
    id = a->getId();
    currentAction = a->getCurrentAction();
    goal = a->getGoal();
    _isWorking = a->isWorking();
    _shouldWorkAtGoal = a->shouldWorkAtGoal();
}
AgentInfo::AgentInfo(AgentInfo&& a){
    id = a.getId();
    currentAction = a.getCurrentAction();
    goal = a.getGoal();
    _isWorking = a.isWorking();
    _shouldWorkAtGoal = a.shouldWorkAtGoal();
}
AgentInfo::AgentInfo(int id, Action currentAction, std::shared_ptr<Vertex> destination, bool isWorking, bool shouldWorkAtGoal){
    this->id = id;
    this->currentAction = currentAction;
    this->goal = destination;
    this->_isWorking = isWorking;
    this->_shouldWorkAtGoal = shouldWorkAtGoal;
}


int AgentInfo::getId() const { return id; }
Action AgentInfo::getCurrentAction() const { return currentAction; }
std::shared_ptr<Vertex> AgentInfo::getGoal() const { return goal; }

void AgentInfo::operator=(const AgentInfo& other){
    id = other.id;
    currentAction = other.currentAction;
    goal = other.goal;
    _isWorking = other._isWorking;
    _shouldWorkAtGoal = other._shouldWorkAtGoal;
}

bool AgentInfo::isWorking(){
    return this->_isWorking;
}

bool AgentInfo::shouldWorkAtGoal(){
    return this->_shouldWorkAtGoal;
}