#ifndef BOX
#define BOX

#include "point.hpp"
#include "line.hpp"

#include "nlohmann/json.hpp"
#include "argos3/plugins/robots/foot-bot/simulator/footbot_entity.h"
#include "argos3/plugins/simulator/entities/box_entity.h"
#include "argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h"

#include <sys/stat.h>
#include <sys/types.h>

#define robotDiameter 0.2f
#define offset_for_obstacles robotDiameter * 1.5

class Box {
private:
  float x, y, h, w;
  argos::CVector3 size, center;
  std::string id;
  std::vector<Point> corners;
  std::vector<Point> virtualCorners;
  std::vector<Line> boxLines;

public:
  Box(std::string id, argos::CVector3 center, argos::CVector3 size);

  //returns the box id
  std::string getID() { return id; }
  //returns virtual corner of the figure
  Point &getVCorner(int n) { return virtualCorners[n]; }
  //returns the actual line of the figure
  Line &getBoxLine(int n) { return boxLines[n]; }
  //build box coners and extends it's virtual corners
  void setBoxCorner();
  const std::vector<std::tuple<float, float, float, float>> getCoordinates();
};

#endif