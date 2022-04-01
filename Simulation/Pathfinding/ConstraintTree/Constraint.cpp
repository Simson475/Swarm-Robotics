#include "Constraint.hpp"

Constraint::Constraint(AgentInfo agent, Location location, float timeStart, float timeEnd){
    this->agent = agent;
    this->location = location;
    this->timeStart = timeStart;
    this->timeEnd = timeEnd;
}


std::string Constraint::toString(){
    return "{agentId: " + std::to_string(agent.getId()) + ", location: " + location.toString() + ", timeStart:" + std::to_string(timeStart) + ", " + "timeEnd:" + std::to_string(timeEnd)+"}";
}