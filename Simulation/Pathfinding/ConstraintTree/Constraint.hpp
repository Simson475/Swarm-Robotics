#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

class Constraint;

#include "AgentInfo.hpp"
#include "Location.hpp"
#include <memory>

class Constraint {
  public:
    Constraint(AgentInfo, Location, uint, uint);
    AgentInfo agent;
    Location location;
    uint timeStart;
    uint timeEnd;
};

#endif