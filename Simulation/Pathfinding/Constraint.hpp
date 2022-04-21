#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

class Constraint;

#include "AgentInfo.hpp"
#include "Location.hpp"
#include <memory>

class Constraint {
  public:
    Constraint(int agentId, Location location, float timeStart, float timeEnd);
    int agentId;
    Location location;
    float timeStart;
    float timeEnd;
    std::string toString() const;
    bool operator==(const Constraint&);
};

#endif