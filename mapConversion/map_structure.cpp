#include "map_structure.h"

void Map_Structure::collectAllWayPoints() {
  for (auto &fig : figures) {
    for (auto &pt : fig->getOffSet()) {
      points.push_back(pt);
    }
  }
}

vector<vector<float>> Map_Structure::createCopyList() {
  auto size = sqrt(Map_Structure::lines.size());
  vector<vector<float>> copyList(size, vector<float>(size));
  for (auto &line : Map_Structure::lines) {
    copyList[line.Geta().lock()->getId()][line.Getb().lock()->getId()] =
        line.GetFloydDistance();
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
  for (auto &line : Map_Structure::lines) {
    copyList[line.Geta().lock()->getId()][line.Getb().lock()->getId()] =
        line.GetFloydDistance();
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
    for (auto i = 0; i < copyList.size(); i++) {
      for (auto j = 0; j < copyList.size(); j++) {
        float temp = copyList[i][k] + copyList[k][j];
        if (copyList[i][k] == INF || copyList[k][j] == INF)
          temp = INF;
        if (copyList[i][k] != INF && copyList[k][j] != INF &&
            temp < copyList[i][j]) {
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
      if (i < amountOfStations && j < amountOfStations)
        shortestDistance[i].push_back(copyList[i][j] / (double)VELOCITY * 100);
      shortestDistances[i].push_back(copyList[i][j] / (double)VELOCITY * 100);
    }
  }
  return shortestDistance;
}

void Map_Structure::createStaticJSON(std::string path) {
  nlohmann::json jsonObj;
  jsonObj["station_delay"] = 6;
  jsonObj["waypoint_delay"] = 3;
  jsonObj["uncertainty"] = 1.1;
  vector<int> dropIds;
  vector<int> contIds;
  for (auto &p : Map_Structure::stations) {
    if (p->getType() == Type::endpoint) {
      dropIds.push_back(p->getId());
    }
    if (p->getType() == Type::station) {
      contIds.push_back(p->getId());
    }
  }
  jsonObj["end_stations"] = dropIds;

  jsonObj["station_distance_matrix"] =
      floydShortest(contIds.size() + dropIds.size());

  jsonObj["stations"] = contIds;
  vector<int> vias;
  for (auto i = 0; i < Map_Structure::points.size(); i++)
    if (Map_Structure::points[i].lock()->getType() == 0)
      vias.push_back(Map_Structure::points[i].lock()->getId());
  jsonObj["vias"] = vias;
  // collect all the wayPoints in 2 dimensions
  int sizeLines = sqrt(Map_Structure::lines.size());
  vector<vector<float>> waypointsDistances(sizeLines, vector<float>());
  int k = -1;
  for (auto i = 0; i < Map_Structure::lines.size(); i++) {
    if (i % sizeLines == 0)
      k++;
    if (Map_Structure::lines[i].GetDistance() == -1) {
      waypointsDistances[k].push_back(Map_Structure::lines[i].GetDistance());
    } else
      waypointsDistances[k].push_back(Map_Structure::lines[i].GetDistance() /
                                      (double)VELOCITY * 100);
  }
  jsonObj["waypoint_distance_matrix"] = waypointsDistances;

  // creation of waypoints json Object
  int j = 0;
  for (auto i = 0; i < Map_Structure::points.size(); i++) {
    for (j; j < Map_Structure::lines.size(); j++) {
      if (Map_Structure::points[i].lock()->getId() ==
          Map_Structure::lines[j].Geta().lock()->getId()) {
        if (Map_Structure::lines[j].GetDistance() != -1)
          Map_Structure::points[i].lock()->pushAdjID(
              Map_Structure::lines[j].Getb().lock()->getId());
      } else
        break;
    }
  }
  vector<nlohmann::json> waypoints;
  for (auto i = 0; i < Map_Structure::points.size(); i++) {
    nlohmann::json wayPoint;
    wayPoint["adjList"] = Map_Structure::points[i].lock()->getAdjIDs();
    wayPoint["id"] = Map_Structure::points[i].lock()->getId();
    switch (Map_Structure::points[i].lock()->getType()) {
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
    wayPoint["x"] = Map_Structure::points[i].lock()->getX();
    wayPoint["y"] = Map_Structure::points[i].lock()->getY();
    waypoints.push_back(wayPoint);
  }
  jsonObj["waypoints"] = waypoints;
  string tmp = path + "/";
  std::ofstream out(tmp + "static_config.json");
  out << std::setw(4) << jsonObj;
  out.close();
}
// Combines from all the points all possibleMap_Structure::lines
void Map_Structure::setAllPossibleLines() {
  // collect all possible lines between off set points
  for (auto i = 0; i < figures.size(); i++) {
    for (auto e = 0; e < figures[i]->getOffSet().size(); e++) {
      for (auto j = 0; j < figures.size(); j++) {
        if (j != i) {
          for (auto k = 0; k < figures[j]->getOffSet().size(); k++) {
            Map_Structure::lines.push_back(
                Line(Map_Structure::figures[i]->getOffSet()[e],
                     Map_Structure::figures[j]->getOffSet()[k]));
          }
        }
      }
    }
    for (auto &line : figures[i]->getOffSetLines()) {
      hardLines.push_back(line);
    }
  }

  // check if stations belongs to any of the figures
  for (auto &fig : figures) {
    for (auto &station : stations) {

      if (fig->isInside(*station)) {
        station->setParent(fig);
      }
    }
  }
  // collect all the possible lines between stations and off set points
  std::vector<std::vector<Line>> stationLines(stations.size(), vector<Line>());
  ;
  int k = 0;
  for (auto &station : stations) {
    if (station->getFigure() != nullptr) {
      for (auto &point : points) {
        if (station != point.lock()) {
          stationLines[k].push_back(Line(station, point));
          stationLines[k].push_back(Line(point, station));
        }
      }
      for (auto &line : stationLines[k]) {
        for (auto &point : station->getFigure()->getPoints()) {
          if (line.ContainsPoint(point, 7)) {
            line.setFailureline();
            line.SetDistance(-1);
            line.setTime(-1);
            break;
          }
        }
      }
      // collect the only hard lines for station which is inside figure
      std::vector<std::weak_ptr<Line>> hardLinesForStation;
      for (auto &fig : figures) {
        if (fig != station->getFigure()) {
          for (auto &p : fig->getOffSetLines())
            hardLinesForStation.push_back(p);
        }
      }
      eliminateBadLines(hardLinesForStation, stationLines[k]);
      k++;
    } else {
      // if station does not belong to figure we simply add them
      for (auto &point : points) {
        Map_Structure::lines.push_back(Line(station, point));
        Map_Structure::lines.push_back(Line(point, station));
      }
    }
  }
  // elimination of lines which intersect with a rough surface
  for (auto i = 0; i < stations.size(); i++) {
    for (auto j = 0; j < stations.size(); j++) {
      lines.push_back(Line(stations[i], stations[j]));
    }
  }
  eliminateBadLines(hardLines, lines);
  for (auto &sLines : stationLines) {
    for (auto &sLine : sLines) {
      lines.push_back(std::move(sLine));
    }
  }
  // collect all the possible lines from each figure
  for (auto &fig : figures) {
    for (auto i = 0; i < fig->getOffSet().size(); i++) {
      for (auto j = 0; j < fig->getOffSet().size(); j++) {
        if (j == fig->getOffSet().size() - 1 && i == 0 ||
            i == fig->getOffSet().size() - 1 && j == 0) {
          lines.push_back(Line(fig->getOffSet()[i], fig->getOffSet()[j]));
        } else {

          if (i == j + 1 || i == j - 1) {
            lines.push_back(Line(fig->getOffSet()[i], fig->getOffSet()[j]));
          } else {
            Line line = Line(fig->getOffSet()[i], fig->getOffSet()[j]);

            line.setFailureline();
            line.SetDistance(-1);
            line.setTime(-1);
            lines.push_back(line);
          }
        }
      }
    }
  }
  ofstream myfile;
  myfile.open("../Plotting/plot4.dat", std::ios_base::app);
  for (auto &line : lines) {
    if (line.GetDistance() != -1) {
      myfile << line.Geta().lock()->getY() << " " << line.Geta().lock()->getX()
             << "\n";
      myfile << line.Getb().lock()->getY() << " " << line.Getb().lock()->getX()
             << "\n"
             << "\n";
    }
  }
  myfile.close();
}

float cross(Point a, Point b) {
  return a.getX() * b.getY() - a.getY() * b.getX();
}

float dot(Point a, Point b) {
  return a.getX() * b.getX() + a.getY() * b.getY();
}

bool intersectionInterest(Point m1, Point m2, Point n1, Point n2) {
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
void Map_Structure::eliminateBadLines(std::vector<weak_ptr<Line>> &hardLines,
                                      vector<Line> &lines) {
  std::vector<int> adjIDs;
  for (int i = 0; i < lines.size(); i++) {
    for (auto j = 0; j < hardLines.size(); j++) {
      if (intersectionInterest(*lines[i].Geta().lock(), *lines[i].Getb().lock(),
                               *hardLines[j].lock()->Geta().lock(),
                               *hardLines[j].lock()->Getb().lock())) {
        Map_Structure::lines[i].setFailureline();
        lines[i].SetDistance(-1);
        lines[i].setTime(-1);
        break;
      }
    }
    if (i != 0) {
      if (Map_Structure::lines[i].GetDistance() != -1) {
        if (Map_Structure::lines[i].Geta().lock()->getId() ==
            Map_Structure::lines[i - 1].Geta().lock()->getId())
          adjIDs.push_back(Map_Structure::lines[i].Getb().lock()->getId());
        else {

          Map_Structure::lines[i].Geta().lock()->setAdjIDs(adjIDs);
          adjIDs.clear();
          adjIDs.push_back(Map_Structure::lines[i].Getb().lock()->getId());
        }
      }
    } else {
      adjIDs.push_back(Map_Structure::lines[i].Getb().lock()->getId());
    }
  }
}

vector<Point> Map_Structure::findPath(int startId, int destinationId) {
  vector<Point> pts;
  if (startId == destinationId) {
    return pts;
  }
  auto u = startId + 1;
  auto v = destinationId + 1;
  do {
    u = shortestPath[u - 1][v - 1];
  } while (u != v);
  return pts;
}

void Map_Structure::initializeStations(std::string path) {
  // get all the points defined in JSON file
  std::ifstream i(path + "points.json");
  nlohmann::json j = nlohmann::json::parse(i);
  for (auto i = 0; i < j.size(); i++) {
    if (j[i].value("x", 0.0) != 0.0) {
      std::shared_ptr<Point> p(new Point(
          j[i].value("x", 0.0), j[i].value("y", 0.0), j[i].value("z", 0.0),
          static_cast<Type>(j[i].value("type", 0)), j[i].value("name", ""),
          nullptr));
      p->incrementId();
      stations.push_back(std::move(p));
    }
    if (static_cast<Type>(j[i].value("type", 0)) == Type::station)
      stationIDs.push_back(stations[i]->getId());
    if (static_cast<Type>(j[i].value("type", 0)) == Type::endpoint)
      endStationIDs.push_back(stations[i]->getId());
  }
  ofstream myfile;
  myfile.open("../Plotting/plot3.dat", std::ios_base::app);
  for (auto &point : stations) {
    myfile << point->getY() << " " << point->getX() << std::endl;
  }
  myfile.close();
}
    void Figure::removeOffPoint(shared_ptr<Point>& p){
      for (auto it = offsetPoints.begin(); it != offsetPoints.end(); ++it){
        if(*it == p){
          offsetPoints.erase(it);
        }
      }
    }