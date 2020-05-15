#ifndef MAP_STRUCTURE
#define MAP_STRUCTURE
#include "nlohmann/json.hpp"
#include "Models/Figure.hpp"

using namespace std;

class Map_Structure {
public:
  std::vector<std::shared_ptr<Point>> stations;
  std::vector<std::shared_ptr<Point>> points;
  std::vector<Line> lines;
  std::vector<std::shared_ptr<Figure>> figures;
  std::vector<std::shared_ptr<Line>> hardLines;
  //ID's of all stations
  std::vector<int> stationIDs;
  //ID's of all drop points aka end stations
  std::vector<int> endStationIDs;
  //Storage of shortest distances between all vias
  std::vector<std::vector<int>> shortestPath;
  //Storage of shortest distances between all stations
  std::vector<std::vector<float>> shortestDistances;

  //singletone of map_structure
  static Map_Structure &get_instance() {
    static Map_Structure instance;
    return instance;
  }
  //Collects each figure vias
  void collectAllWayPoints();

  // creates static_config json file for Uppaal
  void createStaticJSON(std::string path);

  //calculates and returns shortest distances between all stations as well as 
  //stores all shortest paths between all vias
  vector<vector<float>> floydShortest(int amountOfStations);

  // Combines from all the points all possibleStaticMap::lines
  void setAllPossibleLines();

  // Functions eliminates all theStaticMap::lines which have intersection
  void eliminateBadLines(std::vector<shared_ptr<Line>>& hardLines, vector<Line>& lines);

  //function which return shortest path between two points
  std::vector<Point> findPath(int startId, int destinationId);

  // reads Data/points.json file and initializes the stations
  void initializeStations(std::string path);

  //Assistant function for floyd shortest path
  vector<vector<float>> createCopyList();

  //sorts all the lines in order
  void sortLines();
  
  

private:
  Map_Structure() {}
};
#endif