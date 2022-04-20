#include "Conflict.hpp"

Conflict::Conflict(Conflict* c){
    agentIds = c->agentIds;
    timeStart = c->timeStart;
    timeEnd = c->timeEnd;
    location = c->location;
    type = c->type;
}
Conflict::Conflict(const Conflict& c){
    agentIds = c.agentIds;
    timeStart = c.timeStart;
    timeEnd = c.timeEnd;
    location = c.location;
    type = c.type;
}
Conflict::Conflict(Conflict&& c){
    agentIds = c.agentIds;
    timeStart = c.timeStart;
    timeEnd = c.timeEnd;
    location = c.location;
    type = c.type;
}

Conflict::Conflict(std::vector<int> agentIds, float timeStart, float timeEnd, Location location, std::string type){
    this->agentIds = agentIds;
    this->timeStart = timeStart;
    this->timeEnd = timeEnd;
    this->location = location;
    this->type = type;
}

std::vector<int> Conflict::getAgentIds(){
    return this->agentIds;
}

float Conflict::getTimeStart(){
    return this->timeStart;
}
float Conflict::getTimeEnd(){
    return this->timeEnd;
}
Location Conflict::getLocation(){
    return this->location;
}

std::string Conflict::toString(){
    return "{{" + std::to_string(this->agentIds[0]) + "," + std::to_string(this->agentIds[1]) + "} " + this->location.toString() + " [" + std::to_string(this->timeStart) + "," + std::to_string(this->timeEnd) + "] " + this->type + " }";
}