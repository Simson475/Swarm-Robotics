/*** These files are called _path instead of Path due to CMakeList highlighting Path, which maybe indicates that name as problematic ***/

#ifndef PATH_HPP
#define PATH_HPP

#include <queue>
#include "Action.hpp"

class Path {
  public:
    /* Variables */
    std::vector<Action> actions;
    float cost;

    /* Methods */
    std::vector<int> asWaypointPlan();
    std::string toString();
    Path operator+(const Path& path);
    void operator=(const Path &other);
    void operator=(std::vector<Action> actions);
};

#endif