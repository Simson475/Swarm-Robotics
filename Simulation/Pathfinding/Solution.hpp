#ifndef SOLUTION_HPP
#define SOLUTION_HPP

class Solution;

#include <vector>
#include <memory>
#include "Path.hpp"
#include "AgentInfo.hpp"
#include "GLOBALS.hpp"

class Solution {
  public:
    std::vector<Path> paths;
    void finalize(std::vector<AgentInfo> agents);
};

#endif