#ifndef BOX
#define BOX

#include "point.hpp"
#include "line.hpp"

#include "argos3/core/utility/math/vector3.h"

#include <vector>
#include <tuple>

#define robotDiameter 0.2f
#define offset_for_obstacles robotDiameter * 1.5

class Box {
private:
  float x, y, h, w;
  argos::CVector3 size, center;
  std::string id;
  std::vector<Point> corners;
  std::vector<Point> virtualCorners;
  std::vector<Line> virtualLines;
  std::vector<Line> boxLines;

public:
  Box(std::string id, argos::CVector3 center, argos::CVector3 size);

  //returns the box id
  std::string getID() { return id; }
  //returns virtual corner of the figure
  Point &getVCorner(int n) { return virtualCorners[n]; }
  //returns the actual line of the figure
  Line &getBoxLine(int n) { return boxLines[n]; }
  //build box corners and extends it's virtual corners
  void setBoxCorner();
  bool isPointInShape(Point& p);
  bool isPointPartOfTheBox(Point& p);
  Line& getClosestLineToAPoint(Point &p);
  std::vector<Line>& getVirtualLines() {return virtualLines;}
  const std::vector<std::tuple<float, float, float, float>> getCoordinates();
};

#endif