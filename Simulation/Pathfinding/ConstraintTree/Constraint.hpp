#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

class Constraint;

#include "AgentInfo.hpp"
#include "Location.hpp"
#include <memory>

class Constraint {
  public:
    Constraint(AgentInfo, Location, float, float);
    AgentInfo agent;
    Location location;
    float timeStart;
    float timeEnd;
    std::string toString();
};

#endif