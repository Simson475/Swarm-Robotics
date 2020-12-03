#include "box.hpp"

Box::Box(std::string id, argos::CVector3 center, argos::CVector3 size) {
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

const std::vector<std::tuple<float, float, float, float>> Box::getCoordinates(){
    std::vector<std::tuple<float, float, float, float>> coordinates{};

    for (Line l : boxLines) {
        coordinates.emplace_back(l.getCoordinates());
    }

    return coordinates;
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
  virtualLines.push_back(Line(&virtualCorners[0], &virtualCorners[1]));
  virtualLines.push_back(Line(&virtualCorners[1], &virtualCorners[2]));
  virtualLines.push_back(Line(&virtualCorners[2], &virtualCorners[3]));
  virtualLines.push_back(Line(&virtualCorners[3], &virtualCorners[0]));
  //std::cout << "--------------------------------------------------------------------------------------------------------"<<std::endl;
}
//bool Box::isPointOnShape(Point& p) {
//    if (p.getX() == virtualCorners[2].getX() || p.getX() == virtualCorners[0].getX())
//        if (p.getY() > virtualCorners[2].getY() && p.getY() < virtualCorners[0].getY())
//            return true;
//
//    if (p.getY() == virtualCorners[2].getX() || p.getY() == virtualCorners[0].getY())
//        if (p.getX() > virtualCorners[2].getY() && p.getX() < virtualCorners[0].getX())
//            return true;
//
//
//}
bool Box::isPointInShape(Point& p) {
    bool inX = false;
    bool inY = false;
    std::cout <<virtualCorners[2].getX() << "  " << virtualCorners[2].getY() <<std::endl;
    if (p.getX() > virtualCorners[2].getX() && p.getX() < virtualCorners[0].getX())
        inX = true;

    if (p.getY() > virtualCorners[2].getY() && p.getY() < virtualCorners[0].getY())
        inY = true;
    //isPointOnShape(p);
    return (inX && inY);
}
