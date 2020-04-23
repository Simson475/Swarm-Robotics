#ifndef MAP_ELEMENTS
#define MAP_ELEMENTS
#include "json.hpp"
#include <sys/stat.h>
#include <sys/types.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

using namespace std;
using namespace argos;
#define VELOCITY 100.0f
#define INF 999999
#define robotDiameter 0.2f
#define offset robotDiameter * 1.5
enum Type { via, endpoint, station, realCorner, cStation };
class Point : public CVector3 {
  unsigned int id;
  Type pType;
  static unsigned int id_counter;
  vector<int> adjIDs;
  string name;
  bool occupied = false;

public:
  Point(float x, float y, float z, Type type, string name);
  Point(CVector3 c, Type type, string name);
  ~Point();
  Point(Point && p);
  Point& operator=(Point const& obj);
  Point( const Point &obj);
  Point();
  int getType() const { return pType; }
  string getName() const { return name; }
  int getId() const { return id; }
  void setAdjIDs(vector<int> adjID);
  void pushAdjID(int adjID);
  static void resetIdCount();
  bool isOccupied(){return occupied;}
  void setOccupied(bool occupied){this->occupied = occupied;}
  vector<int> getAdjIDs() const { return adjIDs; }
  CVector3 getCVector() {
    return CVector3(this->GetX(), this->GetY(), this->GetZ());
  }
};
class Line {
  int id;
  Point *a, *b;
  float distance;
  double time;

public:
  Line(){};
  Line(Point *a, Point *b);
  void setFailureline();
  float GetDistance() { return distance; }
  double getTime() { return time; }
  void setTime(double time);
  float getFloydTime();
  float GetFloydDistance();
  void SetDistance(float newDis);
  Point& Geta() { return *a; }
  Point& Getb() { return *b; }
  void draw();
};
class Box {
private:
  float x, y, h, w;
  CVector3 size, center;
  std::string id;


  vector<Point> corners;
  vector<Point> virtualCorners;
  vector<Line> boxLines;

public:
  Box() {}
  Box(std::string id, CVector3 center, CVector3 size);
  void draw();
  string getID() { return id; }
  Point &getVCorner(int n) { return virtualCorners[n]; }
  Line &getBoxLine(int n) { return boxLines[n]; }
  void setBoxCorner();
};

#endif