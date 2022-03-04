#ifndef SOLUTION_HPP
#define SOLUTION_HPP

#include <queue>
#include "Agent.hpp"
#include "Conflict.hpp"

class Solution {
  public:
    std::vector<Agent> agents;
    float cost;
    std::vector<Conflict> getConflicts();

  private:
    std::vector<Conflict> conflicts;
};

#endif