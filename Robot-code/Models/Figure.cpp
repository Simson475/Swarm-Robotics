#include "Figure.hpp"

#include <memory>

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point &p, Point &q, Point &r) {
  int val = (q.getY() - p.getY()) * (r.getX() - q.getX()) -
            (q.getX() - p.getX()) * (r.getY() - q.getY());
  if (val == 0)
    return 0;               // collinear
  return (val > 0) ? 1 : 2; // clock or counter clock wise
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
  std::shared_ptr<Point> savrgCenter = std::make_shared<Point>(
      avrg_X, avrg_Y, 0, Type::center, "", shared_from_this());
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
// First = Slope, Second = Y-Intercept
std::pair<float, float> findsLineFunction(const Point& p1, const Point& p2) {
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
std::shared_ptr<Point> Figure::findIntersection(std::pair<Point, Point> &pair1,
                                           std::pair<Point, Point> &pair2) {
  std::pair<float, float> func1 = findsLineFunction(pair1.first, pair1.second);
  std::pair<float, float> func2 = findsLineFunction(pair2.first, pair2.second);
  float x, y;
  if (func1.first == -99999) {
    x = pair1.first.getX();
    y = func2.first * x + func2.second;
    std::shared_ptr<Point> p = std::make_shared<Point>(
        x, y, 0, Type::via, "", std::shared_ptr<Figure>(shared_from_this()));
    p->incrementId();
    return p;
  }
  if (func2.first == -99999) {
    x = pair2.first.getX();
    y = func1.first * x + func1.second;
    std::shared_ptr<Point> p = std::make_shared<Point>(
        x, y, 0, Type::via, "", std::shared_ptr<Figure>(shared_from_this()));
    p->incrementId();
    return p;
  }
  x = (func2.second - func1.second) / (func1.first - func2.first);
  y = func1.first * x + func1.second;
  std::shared_ptr<Point> p = std::make_shared<Point>(
      x, y, 0, Type::via, "", std::shared_ptr<Figure>(shared_from_this()));
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
    // is to keep track of last visited most counter clock-
    // wise point in q. If any point 'i' is more counter clock-
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
  std::ofstream myfile;
  myfile.open("../Plotting/figures.dat", std::ios_base::app);
  for (auto &point : points) {
    myfile << point.getY() << " " << point.getX() << std::endl;
  }
  myfile.close();

  std::vector<std::pair<Point, Point>> pairPoints;
  for (int i = 0; i < hull.size(); i++) {
    Point p1 = hull[i]; // create copies of point for each line
    Point p2;
    if (i != hull.size() - 1) {
      p2 = hull[i + 1];
    } else
      p2 = hull[0];
    std::pair<Point, Point> pairPt;
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

  std::ofstream myfile2;
  myfile2.open("../Plotting/offset.dat", std::ios_base::app);
  for (auto &pts : offsetPoints) {
    myfile2 << pts->getY() << " " << pts->getX() << "\n";
  }

  myfile2.close();
}
std::pair<Point, Point> Figure::moveLine(const Point& p1, const Point& p2, float d) {
  float r = sqrt(pow(p2.getX() - p1.getX(), 2) + pow(p2.getY() - p1.getY(), 2));
  float delta_X = d / r * (p1.getY() - p2.getY());
  float delta_Y = d / r * (p2.getX() - p1.getX());
  Point p3 = Point(p1.getX() + delta_X, p1.getY() + delta_Y, 0, Type::via, "",
                   shared_from_this());
  Point p4 = Point(p2.getX() + delta_X, p2.getY() + delta_Y, 0, Type::via, "",
                   shared_from_this());
  return std::make_pair(p3, p4);
}
void Figure::connectLines() {
  for (int i = 0; i < offsetPoints.size(); i++) {
    if (i != offsetPoints.size() - 1)
      offSetLines.push_back(
          std::make_shared<Line>(offsetPoints[i], offsetPoints[i + 1]));
    else
      offSetLines.push_back(
          std::make_shared<Line>(offsetPoints[i], offsetPoints[0]));
  }
}

// Given three collinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(Point &p, Point &q, Point &r) {
    return q.getX() <= std::max(p.getX(), r.getX()) &&
           q.getX() >= std::min(p.getX(), r.getX()) &&
           q.getY() <= std::max(p.getY(), r.getY()) &&
           q.getY() >= std::min(p.getY(), r.getY());
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
  // p1, q1 and p2 are collinear and p2 lies on segment p1q1
  if (o1 == 0 && onSegment(p1, p2, q1))
    return true;

  // p1, q1 and p2 are collinear and q2 lies on segment p1q1
  if (o2 == 0 && onSegment(p1, q2, q1))
    return true;

  // p2, q2 and p1 are collinear and p1 lies on segment p2q2
  if (o3 == 0 && onSegment(p2, p1, q2))
    return true;

  // p2, q2 and q1 are collinear and q1 lies on segment p2q2
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
      // If the point 'p' is collinear with line segment 'i-next',
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