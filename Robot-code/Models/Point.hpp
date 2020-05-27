#ifndef POINT
#define POINT

#include <sys/stat.h>
#include <sys/types.h>
#include <math.h> 
#include <vector>
#include <string>
#include <memory>
#include <bits/stdc++.h> 

class Figure;

enum Type { via, endpoint, station, realCorner, cStation, center };
class Point{
  int id;
  float x,y,z;
  Type pType;
  static int id_counter;
  std::vector<int> adjIDs;
  std::string name{};
  std::shared_ptr<Figure> fig = std::shared_ptr<Figure>(nullptr);

public:
  Point(float x, float y, float z, Type type, std::string name, std::shared_ptr<Figure> fig);
  //Point(Point && p)  = default;
  Point& operator=(Point const& obj);
  //Point( const Point &obj) = default;
  Point();
  [[nodiscard]] int getType() const { return pType; }
  [[nodiscard]] std::string getName() const { return name; }
  [[nodiscard]] int getId() const { return id; }
  void setAdjIDs(const std::vector<int>& adjID);
  void pushAdjID(int adjID);
  static void resetIdCount();
  // returns vias which can be accessed directly from this point
  [[nodiscard]] std::vector<int> getAdjIDs() const { return adjIDs; }
  [[nodiscard]] float getX() const{return x;}
  [[nodiscard]] float getY() const{return y;}
  [[nodiscard]] float getZ() const{return z;}
  //return parent figure, to which this point belongs 
  //(if station is within figure offset it is consired as part of a figure)
  std::shared_ptr<Figure> getFigure(){return fig;}
  void setX(float x){this->x = x;}
  void setY(float y){this->y = y;}
  void setParent(std::shared_ptr<Figure> figure){this->fig.swap(figure);}
  void incrementId(){id = id_counter;id_counter++;}
  inline Point operator-(const Point& point) const {
    Point pResult(*this);
    pResult -= point;
    return pResult;
  }
  inline Point& operator-=(const Point& point) {
    x -= point.x;
    y -= point.y;
    z -= point.z;
    return *this;
  }
};
#endif