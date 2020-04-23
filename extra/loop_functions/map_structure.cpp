#include "map_structure.h"

void Map_Structure::collectAllWayPoints() {
  
  CSpace::TMapPerType &tBotMap =
      CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
  int i = 0;
  for (CSpace::TMapPerType::iterator it = tBotMap.begin(); it != tBotMap.end();
       ++it) {
    argos::CFootBotEntity *pcBot = any_cast<CFootBotEntity *>(it->second);
    argos::CVector3 pos = pcBot->GetEmbodiedEntity().GetOriginAnchor().Position;
    Point *p = new Point(pos, Type::via, "S." + pcBot->GetId());
    points.push_back(*p);
    Robots.push_back(Robot(pcBot, p));
  }
  std::vector<Box> walls;
  CSpace::TMapPerType &tBoxMap =
      CLoopFunctions().GetSpace().GetEntitiesByType("box");
  for (CSpace::TMapPerType::iterator it = tBoxMap.begin(); it != tBoxMap.end();
       ++it) {
    CBoxEntity *pcBox = any_cast<CBoxEntity *>(it->second);
    CVector3 pos = pcBox->GetEmbodiedEntity().GetOriginAnchor().Position;
    CVector3 size = pcBox->GetSize();
    string id = it->first;
    Box b;
    switch (id[0]) {
    case 'w': // break; allow break in order to stop considering boundries of
              // arena
      walls.push_back(Box(id, pos, size));
      break;
    case 'c':
      b = Box(id, pos, size);
      Map_Structure::boxes.push_back(b);
      break;
    case 'd':
      b = Box(id, pos, size);
      Map_Structure::boxes.push_back(b);
      break;
    case 'o':
      b = Box(id, pos, size);
      Map_Structure::boxes.push_back(b);
      break;
    }
  }
  for (auto i = 0; i < Map_Structure::boxes.size(); i++) {
    Map_Structure::boxes[i].setBoxCorner();
    for (auto j = 0; j < 4; j++) {
      Map_Structure::points.push_back(move(Map_Structure::boxes[i].getVCorner(j)));
      
      Map_Structure::hardLines.push_back(move(Map_Structure::boxes[i].getBoxLine(j)));
    }
  }
}
int Map_Structure::getRobotById(string id) {
  for (auto i = 0; i < Robots.size(); i++) {
    if (Robots[i].getfootBot()->GetId() == id) {
      return i;
    }
  }
}
vector<vector<float>> Map_Structure::createCopyList(){
  auto size = sqrt(Map_Structure::lines.size());
  vector<vector<float>> copyList(size, vector<float>(size));
  for(auto& line: Map_Structure::lines){
    copyList[line.Geta().getId()][line.Getb().getId()] = line.GetFloydDistance();
  }
  shortestPath.clear();
  shortestPath.resize(size, vector<int>(size));
  for (auto i = 0; i < size; ++i) {
    for (auto j = 0; j < size; ++j) {
      shortestPath[i][j] = 0;
    }
    for (auto j = 0; j < size; ++j) {
      if (i != j) {
        shortestPath[i][j] = j + 1;
      }
    }
  }
  return copyList;
}
vector<vector<float>> Map_Structure::floydShortest(int amountOfStations) {
    auto size = sqrt(Map_Structure::lines.size());
  vector<vector<float>> copyList(size, vector<float>(size));
  for(auto& line: Map_Structure::lines){
    copyList[line.Geta().getId()][line.Getb().getId()] = line.GetFloydDistance();
  }
  shortestPath.clear();
  shortestPath.resize(size, vector<int>(size));
  for (auto i = 0; i < size; ++i) {
    for (auto j = 0; j < size; ++j) {
      shortestPath[i][j] = 0;
    }
    for (auto j = 0; j < size; ++j) {
      if (i != j) {
        shortestPath[i][j] = j + 1;
      }
    }
  }
  for (auto k = 0; k < copyList.size(); k++) {
    for (auto i = 0; i < copyList.size(); i++){
      for (auto j = 0; j < copyList.size(); j++){
        float temp = copyList[i][k] + copyList[k][j]; 
        if(copyList[i][k]==INF || copyList[k][j]== INF)temp = INF; 
        if (copyList[i][k] != INF && copyList[k][j]!= INF && temp < copyList[i][j]){
          copyList[i][j] = (copyList[i][k] + copyList[k][j]);
          shortestPath[i][j] = shortestPath[i][k];
        }
      }
    }
  }
  vector<vector<float>> shortestDistance(amountOfStations, vector<float>());
  shortestDistances.resize(copyList.size(), vector<float>());
  for (auto i = 0; i < copyList.size(); i++) {
      for (auto j = 0; j < copyList.size(); j++) {
          if(i <amountOfStations && j < amountOfStations)
            shortestDistance[i].push_back(copyList[i][j]/(double)VELOCITY*100);
          shortestDistances[i].push_back(copyList[i][j]);
        }
      }
  return shortestDistance;
}
Point& Map_Structure::getPointByID(int id){
    for(auto& p : points){
      if(p.getId() == id){
        return p;
      }
    }
  }
  Point* Map_Structure::getPointPointer(int id){
    for (std::vector<Point>::iterator it = points.begin(); it != points.end(); it++) {
        if ((it->getId()) == id)
            return &*it;   // this is the key, if you really want a pointer
    }

    return NULL;
  }

void Map_Structure::createStaticJSON(std::string path) {
  nlohmann::json jsonObj;
  jsonObj["station_delay"] = 6;
  jsonObj["waypoint_delay"] = 3;
  jsonObj["uncertainty"] = 1.1;
  vector<int> dropIds;
  vector<int> contIds;
  for (Point p : Map_Structure::points) {
    if (p.getType() == Type::endpoint) {
      dropIds.push_back(p.getId());
    }
    if (p.getType() == Type::station) {
      contIds.push_back(p.getId());
    }
  }
  jsonObj["end_stations"] = dropIds;

  jsonObj["station_distance_matrix"] =
      floydShortest(contIds.size() + dropIds.size());

  jsonObj["stations"] = contIds;
  vector<int> vias;
  for (auto i = 0; i < Map_Structure::points.size(); i++)
    if (Map_Structure::points[i].getType() == 0)
      vias.push_back(Map_Structure::points[i].getId());
  jsonObj["vias"] = vias;
  // collect all the wayPoints in 2 dimensions
  int sizeLines = sqrt(Map_Structure::lines.size());
  vector<vector<float>> waypointsDistances(sizeLines, vector<float>());
  int k = -1;
  for (auto i = 0; i < Map_Structure::lines.size(); i++) {
    if (i % sizeLines == 0)
      k++;
      if(Map_Structure::lines[i].GetDistance()==-1){
        waypointsDistances[k].push_back(Map_Structure::lines[i].GetDistance());
      }else waypointsDistances[k].push_back(Map_Structure::lines[i].GetDistance()/(double)VELOCITY*100);
  }
  jsonObj["waypoint_distance_matrix"] = waypointsDistances;

  // creation of waypoints json Object
  int j = 0;
  for (auto i = 0; i < Map_Structure::points.size(); i++) {
    for (j; j < Map_Structure::lines.size(); j++) {
      if (Map_Structure::points[i].getId() ==
          Map_Structure::lines[j].Geta().getId()) {
        if (Map_Structure::lines[j].GetDistance() != -1)
          Map_Structure::points[i].pushAdjID(
              Map_Structure::lines[j].Getb().getId());
      } else
        break;
    }
  }
  vector<nlohmann::json> waypoints;
  for (auto i = 0; i < Map_Structure::points.size(); i++) {
    nlohmann::json wayPoint;
    wayPoint["adjList"] = Map_Structure::points[i].getAdjIDs();
    wayPoint["id"] = Map_Structure::points[i].getId();
    switch (Map_Structure::points[i].getType()) {
    case 0:
      wayPoint["type"] = "vias";
      break;
    case 1:
      wayPoint["type"] = "endpoint";
      break;
    case 2:
      wayPoint["type"] = "station";
      break;
    }
    wayPoint["x"] = Map_Structure::points[i].GetX();
    wayPoint["y"] = Map_Structure::points[i].GetY();
    waypoints.push_back(wayPoint);
  }
  jsonObj["waypoints"] = waypoints;
  // create static_config for each robot
  for (auto i = 0; i < Map_Structure::Robots.size(); i++) {
    string tmp = path +
                 Map_Structure::Robots[i].getfootBot()->GetId() + "/";
    std::ofstream out(tmp + "static_config.json");
   // out << std::setw(4) << jsonObj;
           
    out << jsonObj;
  }
}
// Combines from all the points all possibleMap_Structure::lines
void Map_Structure::setAllPossibleLines() {
  for (auto i = 0; i < Map_Structure::points.size(); i++) {
    for (auto j = 0; j < Map_Structure::points.size(); j++) {
      Map_Structure::lines.push_back(
          Line(&Map_Structure::points[i], &Map_Structure::points[j]));
    }
  }
  eliminateBadLines();
}

float cross(CVector3 a, CVector3 b) {
  return a.GetX() * b.GetY() - a.GetY() * b.GetX();
}

float dot(CVector3 a, CVector3 b) {
  return a.GetX() * b.GetX() + a.GetY() * b.GetY();
}

bool intersectionInterest(CVector3 m1, CVector3 m2, CVector3 n1, CVector3 n2) {
  if (cross((n1 - n2), (m1 - n2)) * cross((n1 - n2), (m2 - n2)) < 0 &&
      cross((m1 - m2), (n1 - m2)) * cross((m1 - m2), (n2 - m2)) < 0) {
    return true;
  }
  if (dot(m2 - n2, m2 - n1) <= 0 && dot(m1 - n2, m1 - n1) <= 0 &&
      cross(m1 - m2, n1 - n2) == 0) {
    return true;
  }
  if (cross(n1 - n2, m1 - n2) * cross(n1 - n2, m2 - n2) < 0 &&
      cross(m2 - m1, n1 - m1) * cross(m2 - m1, n2 - m1) == 0) {
    return true;
  }
  return false;
}
// Functions eliminates all theMap_Structure::lines which have intersection
void Map_Structure::eliminateBadLines() {
  std::vector<int> adjIDs;
  for (int i = 0; i < Map_Structure::lines.size(); i++) {
    for (auto j = 0; j < Map_Structure::hardLines.size(); j++) {
      if (intersectionInterest(Map_Structure::lines[i].Geta(),
                               Map_Structure::lines[i].Getb(),
                               Map_Structure::hardLines[j].Geta(),
                               Map_Structure::hardLines[j].Getb())) {
        Map_Structure::lines[i].setFailureline();
        break;
      }
    }
    if (i != 0) {
      if (Map_Structure::lines[i].GetDistance() != -1) {
        if (Map_Structure::lines[i].Geta().getId() ==
            Map_Structure::lines[i - 1].Geta().getId())
          adjIDs.push_back(Map_Structure::lines[i].Getb().getId());
        else {

          Map_Structure::lines[i].Geta().setAdjIDs(adjIDs);
          adjIDs.clear();
          adjIDs.push_back(Map_Structure::lines[i].Getb().getId());
        }
      }
    } else {
      adjIDs.push_back(Map_Structure::lines[i].Getb().getId());
    }
  }

}

vector<Point> Map_Structure::findPath(int startId, int destinationId) {
    vector<Point> pts;
  if(startId == destinationId){
    return pts;
  }
        auto u = startId + 1;
        auto v = destinationId + 1;
        do {
          u = shortestPath[u - 1][v - 1];
          pts.push_back(getPointByID(u-1));
        } while (u != v);
        return pts;
}
void Map_Structure::initializeStations(std::string path) {
  // get all the points defined in JSON file
  std::ifstream i(path +"points.json");
  nlohmann::json j = nlohmann::json::parse(i);
  for (auto i = 0; i < j.size(); i++) {
    if(j[i].value("x", 0.0)!= 0.0){
      Point p = Point(
        CVector3(j[i].value("x", 0.0), j[i].value("y", 0.0),
                 j[i].value("z", 0.0)),
        static_cast<Type>(j[i].value("type", 0)), j[i].value("name", ""));
    points.push_back(std::move(p));
    }
    if (static_cast<Type>(j[i].value("type", 0)) == Type::station)
      stationIDs.push_back(points[i].getId());
    if (static_cast<Type>(j[i].value("type", 0)) == Type::endpoint)

      endStationIDs.push_back(points[i].getId());
  }
}
void Map_Structure::initializeJobs(std::string path) {
  // get all the points defined in JSON file
  std::ifstream i(path +"jobs.json");
  nlohmann::json j = nlohmann::json::parse(i);
  for (auto i = 0; i < j.size(); i++) {
    jobs.push_back(j[i].value("job", std::vector<int>(0)));
  }
}
void Map_Structure::createFolderForEachRobot(std::string path) {
  for (auto i = 0; i < Robots.size(); i++) {
    string temp = path + Robots[i].getfootBot()->GetId();

    if (mkdir(temp.c_str(), 0777) == -1) {
    }
  }
}