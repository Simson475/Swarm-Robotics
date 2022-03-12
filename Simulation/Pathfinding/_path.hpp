#ifndef PATH_HPP
#define PATH_HPP

#include <queue>
#include "Action.hpp"

class Path {
  public:
    std::vector<Action> actions;
    int cost;
};

#endif