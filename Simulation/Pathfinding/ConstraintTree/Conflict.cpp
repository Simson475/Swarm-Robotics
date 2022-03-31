#include "Conflict.hpp"

Conflict::Conflict(Conflict* c){
    agentIds = c->agentIds;
    timeStart = c->timeStart;
    timeEnd = c->timeEnd;
    location = c->location;
}
Conflict::Conflict(const Conflict& c){
    agentIds = c.agentIds;
    timeStart = c.timeStart;
    timeEnd = c.timeEnd;
    location = c.location;
}
Conflict::Conflict(Conflict&& c){
    agentIds = c.agentIds;
    timeStart = c.timeStart;
    timeEnd = c.timeEnd;
    location = c.location;
}

Conflict::Conflict(std::vector<int> agentIds, int timeStart, int timeEnd, Location location){
    this->agentIds = agentIds;
    this->timeStart = timeStart;
    this->timeEnd = timeEnd;
    this->location = location;
}

std::vector<int> Conflict::getAgentIds(){
    return this->agentIds;
}

int Conflict::getTimeStart(){
    return this->timeStart;
}
int Conflict::getTimeEnd(){
    return this->timeEnd;
}
Location Conflict::getLocation(){
    return this->location;
}