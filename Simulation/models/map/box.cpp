#include "box.hpp"
#include <bits/stdc++.h>
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
  boxLines.emplace_back(&corners[0], &corners[1]);
  boxLines.emplace_back(&corners[1], &corners[2]);
  boxLines.emplace_back(&corners[2], &corners[3]);
  boxLines.emplace_back(&corners[3], &corners[0]);
  virtualLines.emplace_back(&virtualCorners[0], &virtualCorners[1]);
  virtualLines.emplace_back(&virtualCorners[1], &virtualCorners[2]);
  virtualLines.emplace_back(&virtualCorners[2], &virtualCorners[3]);
  virtualLines.emplace_back(&virtualCorners[3], &virtualCorners[0]);
}

bool Box::isPointInShape(Point& p) const {
    bool inX = false;
    bool inY = false;

    if (p.getX() > virtualCorners[2].getX() && p.getX() < virtualCorners[0].getX())
        inX = true;

    if (p.getY() > virtualCorners[2].getY() && p.getY() < virtualCorners[0].getY())
        inY = true;

    return (inX && inY);
}

Line& Box::getClosestLineToAPoint(const Point &p){
    double distance = INT_MIN;
    Line* closestLine = &virtualLines[0];
    for(auto& line : virtualLines){
        double temp = line.distanceToLine(p);
        if (temp > distance){

            distance = temp;
            closestLine = &line;
        }
    }
    return *closestLine;
}

bool Box::isPointPartOfTheBox(Point& p){
    return (std::find(virtualCorners.begin(), virtualCorners.end(), p) != virtualCorners.end());
}
