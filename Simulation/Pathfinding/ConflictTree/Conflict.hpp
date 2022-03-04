#ifndef CONFLICT_HPP
#define CONFLICT_HPP

#include <queue>
#include "Agent.hpp"
#include "../models/map/line.hpp"
#include "../models/map/point.hpp"

class Conflict {
  public:
    std::vector<Agent> agents;
    int timestampStart;
    int timestampEnd;
    S location;
    SType locationType;

  private:
    union {
      Point vertex;
      Line edge;
    };

    enum SType{
      VERTEX,
      EDGE
    };
};

#endif