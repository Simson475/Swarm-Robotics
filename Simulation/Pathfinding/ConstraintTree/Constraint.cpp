#include "Constraint.hpp"

Constraint::Constraint(AgentInfo agent, Location location, uint timeStart, uint timeEnd){
    this->agent = agent;
    this->location = location;
    this->timeStart = timeStart;
    this->timeEnd = timeEnd;
}