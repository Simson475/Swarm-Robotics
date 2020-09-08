#include "Point.hpp"

int Point::id_counter = -1;
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
  name = " ";
  this->pType = via;
  x = 0, y = 0, z = 0;
  fig = nullptr;
}
//Point::Point(Point &&p) noexcept
//    : x(p.x), y(p.y) ,z(p.z), name(p.name), id(p.id),
//      pType(p.pType) {}

//Point::Point(const Point &obj) {
//  pType = obj.pType;
//  id = obj.id;
//  name = obj.name;
//  for(auto& adjID : obj.adjIDs)
//      adjIDs.push_back(adjID);
//  x = obj.x;
//  y = obj.y;
//  z = obj.z;
//  fig = obj.fig;
//}
Point &Point::operator=(const Point &obj) = default;
void Point::setAdjIDs(const std::vector<int>& adjID) {
  for (auto& i : adjID)
    adjIDs.push_back(i);
}
void Point::pushAdjID(int adjID) { adjIDs.push_back(adjID); }
void Point::resetIdCount() { id_counter = 0; }