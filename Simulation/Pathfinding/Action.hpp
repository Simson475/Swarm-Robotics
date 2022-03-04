#ifndef ACTION_HPP
#define ACTION_HPP

#include "../models/map/point.hpp"

class Action {
  public:
    int timestamp;
    Point startVertex;
    Point endVertex;
    float cost;
};

#endif