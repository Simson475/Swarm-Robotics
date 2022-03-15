#ifndef SOLUTION_HPP
#define SOLUTION_HPP

#include <vector>
#include "Agent.hpp"
#include "Conflict.hpp"

class Solution {
  public:
    Solution() = default;
    std::vector<Agent*> agents;
    float cost;
    //std::vector<Conflict> getConflicts();

  private:
    //std::vector<Conflict> conflicts;
};

#endif