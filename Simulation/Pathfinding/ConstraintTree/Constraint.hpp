#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include "AgentInfo.hpp"
#include <memory>

class Constraint {
  public:
    Constraint() = default;
    ~Constraint() = default;
    std::unique_ptr<AgentInfo> agent;
    int timeStart;
    int timeEnd;
};

#endif