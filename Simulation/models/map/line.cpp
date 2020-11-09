#include "line.hpp"

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
    return std::numeric_limits<float>::infinity();
}
void Line::SetDistance(float newDis) { distance = newDis; }
void Line::setTime(double time) {this->time = time; }

const std::tuple<float, float, float, float> Line::getCoordinates(){
    std::tuple<float, float, float, float>coordinates{a->GetX(), a->GetY(), b->GetX(), b->GetY()};

    return coordinates;
}

float Line::GetFloydDistance(){
  if (distance != -1)
    return distance;
  else
    return std::numeric_limits<float>::infinity();
}