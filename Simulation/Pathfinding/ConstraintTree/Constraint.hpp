#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

class Constraint;

#include "Agent.hpp"
#include "Location.hpp"
#include <memory>

class Constraint {
  public:
    Constraint(std::shared_ptr<Agent>, Location, uint, uint);
    std::shared_ptr<Agent> agent;
    Location location;
    uint timeStart;
    uint timeEnd;
};

#endif