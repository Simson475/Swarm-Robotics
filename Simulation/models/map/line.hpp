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
  Line(){};
  Line(Point *a, Point *b);
  //setters
  double getTime() { return time; }
  void setTime(double time);
  void SetDistance(float newDis);
  //sets that such line should not exist (distance -1)
  void setFailureline();
  //getters
  float GetDistance() const { return distance; }
  float getFloydTime();
  float GetFloydDistance();
  Point& Geta() { return *a; }
  Point& Getb() { return *b; }
  //draws the line in the simulation, if such line can exist
  void draw();
  const std::tuple<float, float, float, float> getCoordinates();

};

#endif