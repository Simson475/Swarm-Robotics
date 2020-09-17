#ifndef LINE
#define LINE

#include "nlohmann/json.hpp"
#include "point.hpp"
#include <sys/stat.h>
#include <sys/types.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

//using namespace std;
using namespace argos;

#define VELOCITY 100.0f
#define INF 999999

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
  float GetDistance() { return distance; }
  float getFloydTime();
  float GetFloydDistance();
  Point& Geta() { return *a; }
  Point& Getb() { return *b; }
  //draws the line in the simulation, if such line can exist
  void draw();
};

#endif