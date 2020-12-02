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