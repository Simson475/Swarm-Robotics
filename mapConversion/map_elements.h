#ifndef MAP_ELEMENTS
#define MAP_ELEMENTS
#include <sys/stat.h>
#include <sys/types.h>
#include <math.h> 
#include <vector>
#include <string>
#include <memory>
#include <bits/stdc++.h> 

using namespace std;

#define VELOCITY 100.0f
#define INF 999999
#define robotDiameter 1.5f
#define offset robotDiameter * 3
class Figure;

enum Type { via, endpoint, station, realCorner, cStation, center };
class Point{
  unsigned int id;
  float x,y,z;
  Type pType;
  static unsigned int id_counter;
  std::vector<int> adjIDs;
  std::string name;
  shared_ptr<Figure> fig;

public:
  Point(float x, float y, float z, Type type, std::string name, shared_ptr<Figure> fig);
  Point(Point && p);
  Point& operator=(Point const& obj);
  Point( const Point &obj);
  Point();
  int getType() const { return pType; }
  std::string getName() const { return name; }
  int getId() const { return id; }
  void setAdjIDs(std::vector<int> adjID);
  void pushAdjID(int adjID);
  static void resetIdCount();
  // returns vias which can be accessed directly from this point
  std::vector<int> getAdjIDs() const { return adjIDs; }
  float getX(){return x;}
  float getY(){return y;}
  float getZ(){return z;}
  //return parent figure, to which this point belongs 
  //(if station is within figure offset it is consired as part of a figure)
  shared_ptr<Figure> getFigure(){return fig;}
  void setX(float x){this->x = x;}
  void setY(float y){this->y = y;}
  void setParent(shared_ptr<Figure> fig){this->fig.swap(fig);}
  void incrementId(){id = id_counter; id_counter++;}
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


class Line {
  int id;
  std::weak_ptr<Point> a, b;
  float distance;
  double time;

public:

  Line(){};
  Line(std::weak_ptr<Point> a, std::weak_ptr<Point> b);
  //sets line to be -1, this applies to lines which cross hard objects
  void setFailureline();
  //checks if a given points belongs to this line, not 100% result
  bool ContainsPoint(Point point, double SELECTION_FUZZINESS);
  float GetDistance() { return distance; }
  double getTime() { return time; }
  void setTime(double time);
  float getFloydTime();
  float GetFloydDistance();
  void SetDistance(float newDis);
  //checks if a given points belongs to this line  without fuzziness, thus it's more precise
  bool pointBelongsToLine(Point& p);
  std::weak_ptr<Point> Geta() { return a; }
  std::weak_ptr<Point> Getb() { return b; }
  bool operator < (const Line& line) const
    {
        if(line.a.lock()->getId() != a.lock()->getId()){
          return (a.lock()->getId()  < line.a.lock()->getId());
        }else return (b.lock()->getId()  < line.b.lock()->getId());

    }
};


class Figure:public std::enable_shared_from_this<Figure>{
    private:
    //Storage of all rough points of the figure
    std::vector<Point> points;
    //Storate of convex hull a.k.a. in this off set points of figure
    std::vector<std::shared_ptr<Point>> offsetPoints;
    //The center cordinates of this figure
    std::shared_ptr<Point> avrgCenter;
    //final off set lines of this figure
    std::vector<std::shared_ptr<Line>> offSetLines;
    
    public:
    //Empty constructor
    Figure(){};
    //adds hard point to this Figure
    void addPoint(Point& p){points.push_back(p);}
    //simple getters
    std::vector<Point>& getPoints(){return points;}
    std::vector<std::shared_ptr<Line>>& getOffSetLines(){return offSetLines;}
    std::vector<std::shared_ptr<Point>> getOffSet(){return offsetPoints;}
    std::shared_ptr<Point>& getCenter() { return avrgCenter; }
    //Create of this figure convex hull
    void convexHull();
    //Given convex hull finds this Figure's avarage center coordinates
    void findAvrgCenter(std::vector<Point>& hull);
    //Connects all offsets points to get viable lines from them.
    void connectLines();
    //Arranges properly the convex hull
    void cleanConvex(std::vector<Point>& hull);
    //Moves 2 of convex hull points in order to get their off set by distance d ( which should be robot diameter )
    std::pair<Point, Point> moveLine(Point p1, Point p2, float d);
    //Clean ups convex hull, from unnesacary points in order to reduce amount of vias
    void cleanUp(std::vector<Point>& hull);
    //Finds where two lines would intersect if one would extend both of them
    shared_ptr<Point> findIntersection(std::pair<Point, Point>& pair1, std::pair<Point, Point>& pair2);
    //function to check if given point is inside of this Figure
    bool isInside(Point p);
    //removes point from offsetPoints for optimization purposes
    void removeOffPoint(shared_ptr<Point>& p);
};

#endif