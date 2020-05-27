#ifndef LINE
#define LINE

#include <sys/stat.h>
#include <sys/types.h>
#include <math.h> 
#include <vector>
#include <string>
#include <memory>
#include <bits/stdc++.h> 
#include "Point.hpp"

#define VELOCITY 100.0f
#define INF 999999
#define robotDiameter 1.5f
#define offset robotDiameter * 3

class Line {
  std::shared_ptr<Point> a, b;
  float distance{};
  double time{};

public:

  Line()= default;;
  Line(const std::shared_ptr<Point>& a, const std::shared_ptr<Point>& b);
  //sets line to be -1, this applies to lines which cross hard objects
  void setFailureline();
  //checks if a given points belongs to this line, not 100% result
  bool ContainsPoint(const Point& point, double SELECTION_FUZZINESS);
  [[nodiscard]] float GetDistance() const { return distance; }
  [[nodiscard]] double getTime() const { return time; }
  void setTime(double time);
  [[nodiscard]] float getFloydTime() const;
  [[nodiscard]] float GetFloydDistance() const;
  void SetDistance(float newDis);
  //checks if a given points belongs to this line  without fuzziness, thus it's more precise
  bool pointBelongsToLine(Point& p);
  std::shared_ptr<Point> Geta() { return a; }
  std::shared_ptr<Point> Getb() { return b; }
  bool operator < (const Line& line) const
    {
        if(line.a->getId() != a->getId()){
          return (a->getId()  < line.a->getId());
        }else return (b->getId()  < line.b->getId());

    }
};
#endif