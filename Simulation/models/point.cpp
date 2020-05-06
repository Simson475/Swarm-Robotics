#include "point.hpp"

unsigned int Point::id_counter = 0;
Point::Point(float x, float y, float z, Type type, string name)
    : argos::CVector3(x, y, z) {
      
  id = id_counter++;
  
  this->pType = type;
  this->name = name;
}
Point::Point(CVector3 c, Type type, string name)
    : CVector3(c.GetX(), c.GetY(), c.GetZ()) {
  id = id_counter++;
  this->pType = type;
  this->name = name;
}
Point::Point() : CVector3() {
  id = id_counter++;
  this->pType = via;
}
 Point::Point(Point && p) : 
              name(std::move(p.name)),
              id(std::move(p.id)),
              adjIDs(std::move(p.adjIDs)),
              argos::CVector3(p.GetX(), p.GetY(), p.GetZ()),
              pType(std::move(p.pType)){
    }
Point::Point( const Point &obj):
argos::CVector3(obj.GetX(), obj.GetY(), obj.GetZ()){
  pType = obj.pType;
  id = obj.id;
  name = obj.name;
  adjIDs = obj.adjIDs;
  
}
Point& Point::operator=(const Point &obj){
  argos::CVector3(obj.GetX(), obj.GetY(), obj.GetZ());
  pType = obj.pType;
  id = obj.id;
  name = obj.name;
  adjIDs = obj.adjIDs;
  SetX(obj.GetX());
  SetY(obj.GetY());
  SetZ(obj.GetZ());
  
}
Point::~Point(){}
void Point::setAdjIDs(vector<int> adjID) {
  for (auto i = 0; i < adjID.size(); i++) {

    adjIDs.push_back(adjID[i]);
  }
}
void Point::pushAdjID(int adjID) { adjIDs.push_back(adjID); }
void Point::resetIdCount() { id_counter = 0; }
