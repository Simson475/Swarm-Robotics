#ifndef CONFLICT_HPP
#define CONFLICT_HPP

#include <queue>
#include "Agent.hpp"
#include "line.hpp"
#include "point.hpp"

class Conflict
{
private:
  union S
  {
    Point vertex;
    Line edge;
  };

  enum SType
  {
    VERTEX,
    EDGE
  };

public:
  std::vector<Agent*> agents;
  int timestampStart;
  int timestampEnd;
  S location;
  SType locationType;
  Conflict() = default;
  Conflict(Conflict*);
  Conflict(const Conflict&);
  Conflict(Conflict&&) noexcept;
  ~Conflict();
};

#endif