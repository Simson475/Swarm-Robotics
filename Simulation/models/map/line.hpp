#ifndef LINE
#define LINE

#include "point.hpp"

#include "nlohmann/json.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <limits>
#include <tuple>

#define VELOCITY 100.0f

class Line {
  Point *a, *b;
  float distance;
  double time;

public:
  //constructors
  Line(Point *a, Point *b);
  //sets that such line should not exist (distance -1)
  void setFailureline();
  //getters
  float GetDistance() const { return distance; }
  float GetFloydDistance() const;
  Point& Geta() { return *a; }
  Point& Getb() { return *b; }
  double distanceToLine(const Point& point);
  //Get coordinates for drawing in the simulation.
  const std::tuple<float, float, float, float> getCoordinates();
  bool operator==(const Line &l);
};

#endif