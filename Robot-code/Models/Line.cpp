#include "Line.hpp"

Line::Line(const std::shared_ptr<Point>& a, const std::shared_ptr<Point>& b) {
  this->a = a;
  this->b = b;
  // distance between two points
  float flatX = pow((b->getX() - a->getX()), 2);
  float flatY = pow((b->getY() - a->getY()), 2);
  this->distance = std::sqrt(flatX + flatY);

  time = (distance * 100) / VELOCITY;
}
void Line::setFailureline() {
  if (a->getId() != b->getId()) {
    this->distance = -1;
    this->time = -1;
  } else
    this->distance = 0;
}
float Line::getFloydTime() const {
  if (time != -1)
    return time;
  else
    return INF;
}
void Line::SetDistance(float newDis) {
  if (a->getId() != b->getId()) {
    distance = newDis;
  }
}
void Line::setTime(double time) {
  if (a->getId() != b->getId()) {
    time = time;
  }
}
float Line::GetFloydDistance() const {
  if (distance != -1)
    return distance;
  else
    return INF;
}
// First = Slope, Second = Y-Intercept
std::pair<float, float> findLineFunction(const Point& p1, const Point& p2) {
  float m; // slope of the line
  if (p2.getX() != p1.getX()) {
    m = (p2.getY() - p1.getY()) / (p2.getX() - p1.getX());
  } else {
    return std::make_pair(-99999, -99999);
  }
  float b; // the y-intercept
  b = p2.getY() - m * p2.getX();
  return std::make_pair(m, b);
}
// const double SELECTION_FUZZINESS = 7  ;

bool Line::ContainsPoint(const Point& point, double SELECTION_FUZZINESS) {

  // LineGeometry lineGeo = geometry as LineGeometry;
  Point leftPoint;
  Point rightPoint;

  // Normalize start/end to left right to make the offset calc simpler.
  if (a->getX() <= b->getX()) {
    leftPoint = *a;
    rightPoint = *b;
  } else {
    leftPoint = *b;
    rightPoint = *a;
  }
  if (point.getX() < leftPoint.getX() || point.getX() > rightPoint.getX())
    return false;

  float min_Y = std::min(a->getY(), b->getY());
  float max_Y = std::max(a->getY(), b->getY());
  if (point.getY() < min_Y || point.getY() > max_Y)
    return false;

  // return false;
  // If point is out of bounds, no need to do further checks.
  if (point.getX() + SELECTION_FUZZINESS < leftPoint.getX() || rightPoint.getX() < point.getX() - SELECTION_FUZZINESS)
    return false;
  else if (point.getY() + SELECTION_FUZZINESS <
               std::min(leftPoint.getY(), rightPoint.getY()) ||
           std::max(leftPoint.getY(), rightPoint.getY()) <
               point.getY() - SELECTION_FUZZINESS)
    return false;

  double deltaX = rightPoint.getX() - leftPoint.getX();
  double deltaY = rightPoint.getY() - leftPoint.getY();

  // If the line is straight, the earlier boundary check is enough to determine
  // that the point is on the line. Also prevents division by zero exceptions.
  if (deltaX == 0 || deltaY == 0)
    return true;

  double slope = deltaY / deltaX;
  double offseto = leftPoint.getY() - leftPoint.getX() * slope;
  double calculatedY = point.getX() * slope + offseto;
  // Check calculated Y matches the points Y coord with some easing.
  bool lineContains = point.getY() - SELECTION_FUZZINESS <= calculatedY &&
                      calculatedY <= point.getY() + SELECTION_FUZZINESS;

  return lineContains;
}
bool Line::pointBelongsToLine(Point &p) {

  std::pair<float, float> lineFunc = findLineFunction(*a, *b);
  float x, y;
  if (lineFunc.first != -99999) {
    float decider = (lineFunc.first * p.getX() + lineFunc.second) - p.getY();
      return decider == 0;
  } else {
      return p.getX() == a->getX() || p.getY() == a->getY();
  }
}
