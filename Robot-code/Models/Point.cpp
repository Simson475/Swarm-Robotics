#include "Point.hpp"

unsigned int Point::id_counter = 0;
Point::Point(float x, float y, float z, Type type, std::string name,
             std::shared_ptr<Figure> fig) {

  id = -1;

  this->pType = type;
  this->name = name;
  this->x = x;
  this->y = y;
  this->z = z;
  this->fig = fig;
}
Point::Point() {
  id = -1;
  this->pType = via;
  x, y, z = 0;
}
Point::Point(Point &&p)
    : name(std::move(p.name)), id(std::move(p.id)), adjIDs(std::move(p.adjIDs)),
      pType(std::move(p.pType)) {}
Point::Point(const Point &obj) {
  pType = obj.pType;
  id = obj.id;
  name = obj.name;
  adjIDs = obj.adjIDs;
  x = obj.x;
  y = obj.y;
  z = obj.z;
  fig = obj.fig;
}
Point &Point::operator=(const Point &obj) {
  pType = obj.pType;
  id = obj.id;
  name = obj.name;
  adjIDs = obj.adjIDs;
  x = obj.x;
  y = obj.y;
  z = obj.z;
  fig = obj.fig;
  return *this;
}
void Point::setAdjIDs(std::vector<int> adjID) {
  for (auto i = 0; i < adjID.size(); i++) {

    adjIDs.push_back(adjID[i]);
  }
}
void Point::pushAdjID(int adjID) { adjIDs.push_back(adjID); }
void Point::resetIdCount() { id_counter = 0; }