#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include "Agent.hpp"

class Constraint {
  public:
    Agent agent;
    int timestampStart;
    int timestampEnd;
};

#endif