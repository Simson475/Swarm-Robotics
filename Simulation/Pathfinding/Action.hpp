#ifndef ACTION_HPP
#define ACTION_HPP

#include "models/map/map_structure.hpp"
#include "point.hpp"

class Action {
  public:
    int timestamp;
    Vertex startVertex;
    Vertex endVertex;
    float cost;
};

#endif