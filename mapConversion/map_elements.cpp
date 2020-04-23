#include "map_elements.h"

unsigned int Point::id_counter = 0;
Point::Point(float x, float y, float z, Type type, std::string name,
             shared_ptr<Figure> fig) {

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

Line::Line(std::weak_ptr<Point> a, std::weak_ptr<Point> b) {
  this->a = a;
  this->b = b;
  // distance between two points
  float flatX = pow((b.lock()->getX() - a.lock()->getX()), 2);
  float flatY = pow((b.lock()->getY() - a.lock()->getY()), 2);
  this->distance = std::sqrt(flatX + flatY);

  time = (distance * 100) / VELOCITY;
}
void Line::setFailureline() {
  if (a.lock()->getId() != b.lock()->getId()) {
    this->distance = -1;
    this->time = -1;
  } else
    this->distance = 0;
}
float Line::getFloydTime() {
  if (time != -1)
    return time;
  else
    return INF;
}
void Line::SetDistance(float newDis) {
  if (a.lock()->getId() != b.lock()->getId()) {
    distance = newDis;
  }
}
void Line::setTime(double time) {
  if (a.lock()->getId() != b.lock()->getId()) {
    time = time;
  }
}
float Line::GetFloydDistance() {
  if (distance != -1)
    return distance;
  else
    return INF;
}
// First = Slope, Second = Y-Intercept
std::pair<float, float> findLineFunction(Point p1, Point p2) {
  float m; // slope of the line
  if (p2.getX() != p1.getX()) {
    m = (p2.getY() - p1.getY()) / (p2.getX() - p1.getX());
  } else {
    return make_pair(-99999, -99999);
  }
  float b; // the y-intercept
  b = p2.getY() - m * p2.getX();
  return make_pair(m, b);
}
// const double SELECTION_FUZZINESS = 7  ;

bool Line::ContainsPoint(Point point, double SELECTION_FUZZINESS) {

  // LineGeometry lineGeo = geometry as LineGeometry;
  Point leftPoint;
  Point rightPoint;

  // Normalize start/end to left right to make the offset calc simpler.
  if (a.lock()->getX() <= b.lock()->getX()) {
    leftPoint = *a.lock();
    rightPoint = *b.lock();
  } else {
    leftPoint = *b.lock();
    rightPoint = *a.lock();
  }
  if (point.getX() < leftPoint.getX() || point.getX() > rightPoint.getX())
    return false;

  float min_Y = min(a.lock()->getY(), b.lock()->getY());
  float max_Y = max(a.lock()->getY(), b.lock()->getY());
  if (point.getY() < min_Y || point.getY() > max_Y)
    return false;

  // return false;
  // If point is out of bounds, no need to do further checks.
  if (point.getX() + SELECTION_FUZZINESS < leftPoint.getX() ||
      rightPoint.getX() < point.getX() - SELECTION_FUZZINESS)
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

  std::pair<float, float> lineFunc = findLineFunction(*a.lock(), *b.lock());
  float x, y;
  if (lineFunc.first != -99999) {
    float decider = (lineFunc.first * p.getX() + lineFunc.second) - p.getY();
    if (decider == 0) {
      return true;
    } else
      return false;
  } else {
    if (p.getX() == a.lock()->getX() || p.getY() == a.lock()->getY()) {
      return true;
    } else
      return false;
  }
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point &p, Point &q, Point &r) {
  int val = (q.getY() - p.getY()) * (r.getX() - q.getX()) -
            (q.getX() - p.getX()) * (r.getY() - q.getY());
  if (val == 0)
    return 0;               // colinear
  return (val > 0) ? 1 : 2; // clock or counterclock wise
}
void Figure::cleanConvex(std::vector<Point> &hull) {
  std::vector<int> idToDelete;
  int y_min, y_max;
  for (int i = 0; i < hull.size(); i++) {
    y_min = i;
    y_max = i + 1;
    if (y_max + 1 == hull.size())
      break;
    while (hull[y_min].getX() == hull[y_max].getX()) {
      if (hull[y_min].getX() == hull[y_max + 1].getX()) {
        idToDelete.push_back(y_max);
      }
      y_max++;
      if (y_max + 1 == hull.size())
        break;
    }
  }

  int x_min, x_max;
  for (int i = 0; i < hull.size(); i++) {
    x_min = i;
    x_max = i + 1;
    if (x_max + 1 == hull.size())
      break;
    while (hull[x_min].getY() == hull[x_max].getY()) {
      if (hull[x_min].getY() == hull[x_max + 1].getY()) {
        idToDelete.push_back(x_max);
      }
      x_max++;
      if (x_max + 1 >= hull.size())
        break;
    }
  }
  sort(idToDelete.begin(), idToDelete.end());
  idToDelete.erase(unique(idToDelete.begin(), idToDelete.end()),
                   idToDelete.end());
  for (int i = idToDelete.size() - 1; i >= 0; i--) {
    hull.erase(std::begin(hull) + idToDelete[i]);
  }
}
void Figure::findAvrgCenter(std::vector<Point> &hull) {
  float avrg_X = 0;
  float avrg_Y = 0;
  for (auto &point : hull) {
    avrg_X += point.getX();
    avrg_Y += point.getY();
  }
  avrg_X = avrg_X / hull.size();
  avrg_Y = avrg_Y / hull.size();
  shared_ptr<Point> savrgCenter = shared_ptr<Point>(
      new Point(avrg_X, avrg_Y, 0, Type::center, "", shared_from_this()));
  avrgCenter.reset(
      new Point(avrg_X, avrg_Y, 0, Type::center, "", shared_from_this()));
}

float checkSlope(Point &p1, Point &p2) {
  return (p2.getY() - p1.getY()) / (p2.getX() - p1.getX());
}
void Figure::cleanUp(std::vector<Point> &hull) {
  std::vector<int> idToDelete;
  for (int i = 0; i < hull.size() - 2; i++) {
    if (checkSlope(hull[i], hull[i + 1]) == checkSlope(hull[i], hull[i + 2])) {
      idToDelete.push_back(i + 1);
    }
  }
  sort(idToDelete.begin(), idToDelete.end());
  idToDelete.erase(unique(idToDelete.begin(), idToDelete.end()),
                   idToDelete.end());
  for (int i = idToDelete.size() - 1; i >= 0; i--) {
    hull.erase(std::begin(hull) + idToDelete[i]);
  }
}
shared_ptr<Point> Figure::findIntersection(std::pair<Point, Point> &pair1,
                                           std::pair<Point, Point> &pair2) {
  std::pair<float, float> func1 = findLineFunction(pair1.first, pair1.second);
  std::pair<float, float> func2 = findLineFunction(pair2.first, pair2.second);
  float x, y;
  if (func1.first == -99999) {
    x = pair1.first.getX();
    y = func2.first * x + func2.second;
    shared_ptr<Point> p = shared_ptr<Point>(new Point(
        x, y, 0, Type::via, "", shared_ptr<Figure>(shared_from_this())));
    p->incrementId();
    return p;
  }
  if (func2.first == -99999) {
    x = pair2.first.getX();
    y = func1.first * x + func1.second;
    shared_ptr<Point> p = shared_ptr<Point>(new Point(
        x, y, 0, Type::via, "", shared_ptr<Figure>(shared_from_this())));
    p->incrementId();
    return p;
  }
  x = (func2.second - func1.second) / (func1.first - func2.first);
  y = func1.first * x + func1.second;
  shared_ptr<Point> p = shared_ptr<Point>(new Point(
      x, y, 0, Type::via, "", shared_ptr<Figure>(shared_from_this())));
  p->incrementId();
  return p;
}
bool isLeft(Point &a, Point &b, std::shared_ptr<Point> &c) {
  return ((b.getX() - a.getX()) * (c->getY() - a.getY()) -
          (b.getY() - a.getY()) * (c->getX() - a.getX())) > 0;
}
void Figure::convexHull() {
  int n = points.size();
  // There must be at least 3 points
  if (n < 3)
    return;
  std::vector<Point> hull;
  // Find the leftmost point
  int l = 0;
  for (int i = 1; i < n; i++)
    if (points[i].getX() < points[l].getX())
      l = i;
  // Start from leftmost point, keep moving counterclockwise
  // until reach the start point again.  This loop runs O(h)
  // times where h is number of points in result or output.
  int p = l, q;
  do {
    Point pt = Point(points[p].getX(), points[p].getY(), 0, Type::via, "",
                     shared_from_this());
    // Add current point to result
    hull.push_back(pt);

    // Search for a point 'q' such that orientation(p, x,
    // q) is counterclockwise for all points 'x'. The idea
    // is to keep track of last visited most counterclock-
    // wise point in q. If any point 'i' is more counterclock-
    // wise than q, then update q.
    q = (p + 1) % n;
    for (int i = 0; i < n; i++) {
      // If i is more counterclockwise than current q, then
      // update q
      if (orientation(points[p], points[i], points[q]) == 2)
        q = i;
    }
    // Now q is the most counterclockwise with respect to p
    // Set p as q for next iteration, so that q is added to
    // result 'hull'
    p = q;

  } while (p != l); // While we don't come to first point
  findAvrgCenter(hull);
  cleanConvex(hull);
  cleanUp(hull);
  ofstream myfile;
  myfile.open("../Plotting/plot.dat", std::ios_base::app);
  for (auto &point : points) {
    myfile << point.getY() << " " << point.getX() << std::endl;
  }
  myfile.close();

  std::vector<pair<Point, Point>> pairPoints;
  for (int i = 0; i < hull.size(); i++) {
    Point p1 = hull[i]; // create copies of point for each line
    Point p2;
    if (i != hull.size() - 1) {
      p2 = hull[i + 1];
    } else
      p2 = hull[0];
    pair<Point, Point> pairPt;
    if (isLeft(p1, p2, avrgCenter)) {
      pairPt = moveLine(p1, p2, offset * -1);
    } else
      pairPt = moveLine(p1, p2, offset);
    pairPoints.push_back(pairPt);
  }
  for (int i = 0; i < pairPoints.size(); i++) {

    Point p;
    if (i == pairPoints.size() - 1)
      offsetPoints.push_back(findIntersection(pairPoints[i], pairPoints[0]));
    else
      offsetPoints.push_back(
          findIntersection(pairPoints[i], pairPoints[i + 1]));
  }
  connectLines();

  ofstream myfile2;
  myfile2.open("../Plotting/plot2.dat", std::ios_base::app);
  for (auto &pts : offsetPoints) {
    myfile2 << pts->getY() << " " << pts->getX() << "\n";
  }

  myfile2.close();
}
std::pair<Point, Point> Figure::moveLine(Point p1, Point p2, float d) {
  float r = sqrt(pow(p2.getX() - p1.getX(), 2) + pow(p2.getY() - p1.getY(), 2));
  float delta_X = d / r * (p1.getY() - p2.getY());
  float delta_Y = d / r * (p2.getX() - p1.getX());
  Point p3 = Point(p1.getX() + delta_X, p1.getY() + delta_Y, 0, Type::via, "",
                   shared_from_this());
  Point p4 = Point(p2.getX() + delta_X, p2.getY() + delta_Y, 0, Type::via, "",
                   shared_from_this());
  return make_pair(p3, p4);
}
void Figure::connectLines() {
  for (int i = 0; i < offsetPoints.size(); i++) {
    if (i != offsetPoints.size() - 1)
      offSetLines.push_back(
          shared_ptr<Line>(new Line(offsetPoints[i], offsetPoints[i + 1])));
    else
      offSetLines.push_back(
          shared_ptr<Line>(new Line(offsetPoints[i], offsetPoints[0])));
  }
}

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(Point &p, Point &q, Point &r) {
  if (q.getX() <= max(p.getX(), r.getX()) &&
      q.getX() >= min(p.getX(), r.getX()) &&
      q.getY() <= max(p.getY(), r.getY()) &&
      q.getY() >= min(p.getY(), r.getY()))
    return true;
  return false;
}

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(Point &p1, Point &q1, Point &p2, Point &q2) {
  // Find the four orientations needed for general and
  // special cases
  int o1 = orientation(p1, q1, p2);
  int o2 = orientation(p1, q1, q2);
  int o3 = orientation(p2, q2, p1);
  int o4 = orientation(p2, q2, q1);

  // General case
  if (o1 != o2 && o3 != o4)
    return true;

  // Special Cases
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1
  if (o1 == 0 && onSegment(p1, p2, q1))
    return true;

  // p1, q1 and p2 are colinear and q2 lies on segment p1q1
  if (o2 == 0 && onSegment(p1, q2, q1))
    return true;

  // p2, q2 and p1 are colinear and p1 lies on segment p2q2
  if (o3 == 0 && onSegment(p2, p1, q2))
    return true;

  // p2, q2 and q1 are colinear and q1 lies on segment p2q2
  if (o4 == 0 && onSegment(p2, q1, q2))
    return true;

  return false; // Doesn't fall in any of the above cases
}

// Returns true if the point p lies inside the polygon[] with n vertices
bool Figure::isInside(Point p) {
  int n = offsetPoints.size();
  // There must be at least 3 vertices in polygon[]
  if (n < 3)
    return false;

  // Create a point for line segment from p to infinite
  Point extreme = Point(INF, p.getY(), 0, Type::via, "", shared_from_this());

  // Count intersections of the above line with sides of polygon
  int count = 0, i = 0;
  do {
    int next = (i + 1) % n;

    // Check if the line segment from 'p' to 'extreme' intersects
    // with the line segment from 'polygon[i]' to 'polygon[next]'
    if (doIntersect(*offsetPoints[i], *offsetPoints[next], p, extreme)) {
      // If the point 'p' is colinear with line segment 'i-next',
      // then check if it lies on segment. If it lies, return true,
      // otherwise false
      if (orientation(*offsetPoints[i], p, *offsetPoints[next]) == 0)
        return onSegment(*offsetPoints[i], p, *offsetPoints[next]);

      count++;
    }
    i = next;
  } while (i != 0);

  // Return true if count is odd, false otherwise
  return count & 1; // Same as (count%2 == 1)
}