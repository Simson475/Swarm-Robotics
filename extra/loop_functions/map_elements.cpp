#include "map_elements.h"

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

Line::Line(Point *a, Point *b) {
  this->a = a;
  this->b = b;
  //this->distance = (a - b).Length();
  this->distance = argos::Distance(*a,*b);
  time = (distance * 100) / VELOCITY; 
  
}
void Line::setFailureline() {
  if (a->getId() != b->getId()) {
    this->distance = -1;
    this->time = -1;
  }
}

float Line::getFloydTime() {
  if (time != -1)
    return time;
  else
    return INF;
}
void Line::SetDistance(float newDis) { distance = newDis; }
void Line::setTime(double time) {this->time = time; }
void Line::draw() {
  glBegin(GL_LINES);
  glVertex2f(a->GetX(), a->GetY());
  glVertex2f(b->GetX(), b->GetY());
  glEnd();
}
float Line::GetFloydDistance(){
  if (distance != -1)
    return distance;
  else
    return INF;
}
Box::Box(std::string id, CVector3 center, CVector3 size) {
  this->center = center;
  this->id = id;
  this->size = size;

  x = center.GetX();
  y = center.GetY();
  h = size.GetY();
  w = size.GetX();

  virtualCorners.push_back(Point(x + w / 2 + offset, y + h / 2 + offset,
                                 center.GetZ(), Type::via, id + "Vcorner1"));
  virtualCorners.push_back(Point(x - w / 2 - offset, y + h / 2 + offset,
                                 center.GetZ(), Type::via, id + "Vcorner2"));
  virtualCorners.push_back(Point(x - w / 2 - offset, y - h / 2 - offset,
                                 center.GetZ(), Type::via, id + "Vcorner3"));
  virtualCorners.push_back(Point(x + w / 2 + offset, y - h / 2 - offset,
                                 center.GetZ(), Type::via, id + "Vcorner4"));
}
void Box::draw() {
  for (Line l : boxLines) {
    l.draw();
  }
}
void Box::setBoxCorner() {
  corners.push_back(Point(x + w / 2, y + h / 2, center.GetZ(), Type::realCorner,
                          id + "corner1"));
  corners.push_back(Point(x - w / 2, y + h / 2, center.GetZ(), Type::realCorner,
                          id + "corner2"));
  corners.push_back(Point(x - w / 2, y - h / 2, center.GetZ(), Type::realCorner,
                          id + "corner3"));
  corners.push_back(Point(x + w / 2, y - h / 2, center.GetZ(), Type::realCorner,
                          id + "corner4"));
  boxLines.push_back(Line(&corners[0], &corners[1]));
  boxLines.push_back(Line(&corners[1], &corners[2]));
  boxLines.push_back(Line(&corners[2], &corners[3]));
  boxLines.push_back(Line(&corners[3], &corners[0]));
  int i = 0 ;
    for(auto& line : boxLines){
    //std::cout<<"From a: " << line.Geta().getName()<<" to b: "<<line.Getb().getName()<<" distance: "<< line.GetDistance()<< " ITERATION: "<<i++<<std::endl;
  }
  //std::cout << "--------------------------------------------------------------------------------------------------------"<<std::endl;
}
