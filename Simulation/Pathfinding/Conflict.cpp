#include "Conflict.hpp"

Conflict::Conflict(Conflict* c){
    agentIds = c->agentIds;
    timeStart = c->timeStart;
    timeEnd = c->timeEnd;
    locations = c->locations;
    type = c->type;
}
Conflict::Conflict(const Conflict& c){
    agentIds = c.agentIds;
    timeStart = c.timeStart;
    timeEnd = c.timeEnd;
    locations = c.locations;
    type = c.type;
}
Conflict::Conflict(Conflict&& c){
    agentIds = c.agentIds;
    timeStart = c.timeStart;
    timeEnd = c.timeEnd;
    locations = c.locations;
    type = c.type;
}

Conflict::Conflict(std::vector<int> agentIds, float timeStart, float timeEnd, std::vector<Location> locations, std::string type){
    this->agentIds = agentIds;
    this->timeStart = timeStart;
    this->timeEnd = timeEnd;
    this->locations = locations;
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
Location Conflict::getLocation(int index){
    return this->locations[index];
}

std::string Conflict::toString(){
    return "{{" + std::to_string(this->agentIds[0]) + "," + std::to_string(this->agentIds[1]) + "} " + this->locations[0].toString() + " [" + std::to_string(this->timeStart) + "," + std::to_string(this->timeEnd) + "] " + this->type + " }";
}