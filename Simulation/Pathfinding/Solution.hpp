#ifndef SOLUTION_HPP
#define SOLUTION_HPP

#include <vector>
#include "AgentInfo.hpp"
#include "Conflict.hpp"
#include "_path.hpp"

class Solution {
  public:
    std::vector<Path> paths;
    void setPath(AgentInfo&, Path);
};

#endif