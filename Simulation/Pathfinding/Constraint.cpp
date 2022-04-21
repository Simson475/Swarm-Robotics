#include "Constraint.hpp"

Constraint::Constraint(int agentId, Location location, float timeStart, float timeEnd){
    this->agentId = agentId;
    this->location = location;
    this->timeStart = timeStart;
    this->timeEnd = timeEnd;
}


std::string Constraint::toString() const {
    return "{agentId: " + std::to_string(agentId) + ", location: " + location.toString() + ", timeStart:" + std::to_string(timeStart) + ", " + "timeEnd:" + std::to_string(timeEnd)+"}";
}

bool Constraint::operator==(const Constraint& other){
    return agentId == other.agentId &&
            location == other.location &&
            timeStart == other.timeStart &&
            timeEnd == other.timeEnd;
 }