#ifndef AGENT_HPP
#define AGENT_HPP

#include "Path.hpp"

class Agent {
  public:
    Path GetPath();
  private:
    Path path;
    Path FindPath();
};

#endif