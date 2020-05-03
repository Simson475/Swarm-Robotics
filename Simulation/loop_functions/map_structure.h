#ifndef MAP_STRUCTURE
#define MAP_STRUCTURE
#include "json.hpp"

#include "map_elements.h"
#include "robot.h"
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

using namespace std;
using namespace argos;

class Map_Structure {
public:
  std::vector<Point> points;
  std::vector<Line> lines;
  std::vector<Box> boxes;
  std::vector<Line> hardLines;
  std::vector<int> stationIDs;
  std::vector<int> endStationIDs;
  std::vector<Robot> Robots;
  std::vector<std::vector<int>> shortestPath;
  std::vector<std::vector<float>> shortestDistances;
  std::vector<std::vector<int>> jobs;
  int timesUppaalFailed = 0;
  int totalTries = 0;
  static Map_Structure &get_instance() {
    static Map_Structure instance;
    return instance;
  }
  //Map_Structure(Map_Structure const &) = delete;
  //Map_Structure(Map_Structure &&) = delete;
  int getRobotById(string id);
  void collectAllWayPoints();
  void createStaticJSON(std::string path);
  vector<vector<float>> floydShortest(int amountOfStations);
  // Combines from all the points all possibleStaticMap::lines
  void setAllPossibleLines();
  // Functions eliminates all theStaticMap::lines which have intersection
  void eliminateBadLines();
  Point& getPointByID(int id);
  Point* getPointPointer(int id);
  std::vector<Point> findPath(int startId, int destinationId);
  void initializeStations(std::string path);
  void initializeJobs(std::string path);
  void createFolderForEachRobot(std::string path);
  vector<vector<float>> createCopyList(); //function used for shortest path, duplication of data
  //void print(std::vector<std::vector<int>> next, int start, int end);

private:
  Map_Structure() {}
};
#endif