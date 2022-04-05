#include "AgentInfo.hpp"

AgentInfo::AgentInfo(){
    
}
AgentInfo::AgentInfo(const AgentInfo& a){
    id = a.id;
    currentAction = a.currentAction;
    goal = a.goal;
}
AgentInfo::AgentInfo(AgentInfo* a){
    id = a->getId();
    currentAction = a->getCurrentAction();
    goal = a->getGoal();
}
AgentInfo::AgentInfo(AgentInfo&& a){
    id = a.getId();
    currentAction = a.getCurrentAction();
    goal = a.getGoal();
}
AgentInfo::AgentInfo(int id, Action currentAction, std::shared_ptr<Vertex> destination){
    this->id = id;
    this->currentAction = currentAction;
    this->goal = destination;
}

int AgentInfo::getId(){ return id; }
Action AgentInfo::getCurrentAction(){ return currentAction; }
std::shared_ptr<Vertex> AgentInfo::getGoal(){ return goal; }

void AgentInfo::operator=(const AgentInfo& other){
    id = other.id;
    currentAction = other.currentAction;
    goal = other.goal;
}