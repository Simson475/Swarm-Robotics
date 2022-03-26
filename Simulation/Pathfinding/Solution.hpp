#ifndef SOLUTION_HPP
#define SOLUTION_HPP

class Solution;

#include <vector>
#include <memory>
#include "Agent.hpp"
#include "_path.hpp"

class Solution {
  public:
    std::vector<Path> paths;
    void setPath(std::shared_ptr<Agent>, Path);
};

#endif