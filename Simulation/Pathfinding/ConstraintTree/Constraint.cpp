#include "Constraint.hpp"

Constraint::Constraint(std::shared_ptr<Agent> agent, Location location, uint timeStart, uint timeEnd){
    this->agent = agent;
    this->location = location;
    this->timeStart = timeStart;
    this->timeEnd = timeEnd;
}