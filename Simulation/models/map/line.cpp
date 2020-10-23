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