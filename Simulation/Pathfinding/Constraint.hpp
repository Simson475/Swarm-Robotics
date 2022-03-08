#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include "Agent.hpp"

class Constraint {
  public:
    Constraint(Agent a, int start, int end);
    Agent agent;
    int timestampStart;
    int timestampEnd;
};

#endif