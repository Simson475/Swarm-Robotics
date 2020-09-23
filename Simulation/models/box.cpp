#include "box.hpp"

Box::Box(std::string id, CVector3 center, CVector3 size) {
  this->center = center;
  this->id = id;
  this->size = size;

  x = center.GetX();
  y = center.GetY();
  h = size.GetY();
  w = size.GetX();

  virtualCorners.push_back(Point(x + w / 2 + offset_for_obstacles, y + h / 2 + offset_for_obstacles,
                                 center.GetZ(), pointType::via, id + "Vcorner1"));
  virtualCorners.push_back(Point(x - w / 2 - offset_for_obstacles, y + h / 2 + offset_for_obstacles,
                                 center.GetZ(), pointType::via, id + "Vcorner2"));
  virtualCorners.push_back(Point(x - w / 2 - offset_for_obstacles, y - h / 2 - offset_for_obstacles,
                                 center.GetZ(), pointType::via, id + "Vcorner3"));
  virtualCorners.push_back(Point(x + w / 2 + offset_for_obstacles, y - h / 2 - offset_for_obstacles,
                                 center.GetZ(), pointType::via, id + "Vcorner4"));
}
void Box::draw() {
  for (Line l : boxLines) {
    l.draw();
  }
}
void Box::setBoxCorner() {
  corners.push_back(Point(x + w / 2, y + h / 2, center.GetZ(), pointType::realCorner,
                          id + "corner1"));
  corners.push_back(Point(x - w / 2, y + h / 2, center.GetZ(), pointType::realCorner,
                          id + "corner2"));
  corners.push_back(Point(x - w / 2, y - h / 2, center.GetZ(), pointType::realCorner,
                          id + "corner3"));
  corners.push_back(Point(x + w / 2, y - h / 2, center.GetZ(), pointType::realCorner,
                          id + "corner4"));
  boxLines.push_back(Line(&corners[0], &corners[1]));
  boxLines.push_back(Line(&corners[1], &corners[2]));
  boxLines.push_back(Line(&corners[2], &corners[3]));
  boxLines.push_back(Line(&corners[3], &corners[0]));
  //std::cout << "--------------------------------------------------------------------------------------------------------"<<std::endl;
}
