#ifndef AGENT_HPP
#define AGENT_HPP

#include "Path.hpp"
#include "point.hpp"
#include <queue>

class Agent {
  public:
    Path getPath();
    Path createPath(std::vector<Point> &path);
  private:
    Path path;
};

#endif