#ifndef BOX
#define BOX

#include "json.hpp"
#include "point.hpp"
#include "line.hpp"
#include <sys/stat.h>
#include <sys/types.h>

#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

using namespace std;
using namespace argos;

#define robotDiameter 0.2f
#define offset_for_obstacles robotDiameter * 1.5

class Box {
private:
  float x, y, h, w;
  CVector3 size, center;
  std::string id;
  vector<Point> corners;
  vector<Point> virtualCorners;
  vector<Line> boxLines;

public:
  Box() {}
  Box(std::string id, CVector3 center, CVector3 size);
  //draws virtual corners of the box
  void draw();
  //returns the box id
  string getID() { return id; }
  //returns virtual corner of the figure
  Point &getVCorner(int n) { return virtualCorners[n]; }
  //returns the actual line of the figure
  Line &getBoxLine(int n) { return boxLines[n]; }
  //build box coners and extends it's virtual corners
  void setBoxCorner();
};

#endif