#ifndef PATH_HPP
#define PATH_HPP

#include <queue>
#include "Action.hpp"

class Path {
  public:
    /* Variables */
    std::vector<Action*> actions;
    float cost;

    /* Methods */
    std::vector<int> asWaypointPlan();
};

#endif