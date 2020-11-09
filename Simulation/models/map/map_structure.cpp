#include "map_structure.hpp"
#include "models/robot/parsing/uppaal_model_parsing.h"

void Map_Structure::collectAllWayPoints() {

    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto it = tBotMap.begin(); it != tBotMap.end();
         ++it) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(it->second);
        argos::CVector3 pos = pcBot->GetEmbodiedEntity().GetOriginAnchor().Position;

        Point *p = new Point(pos, pointType::via, "S." + pcBot->GetId());
        points.push_back(*p);
        waypointsIDs.push_back(points.end()->getId());
        Robots.push_back(Robot(pcBot, p));
    }
    std::vector<Box> walls;
    argos::CSpace::TMapPerType &tBoxMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("box");
    for (auto it = tBoxMap.begin(); it != tBoxMap.end();
         ++it) {
        argos::CBoxEntity *pcBox = argos::any_cast<argos::CBoxEntity *>(it->second);
        argos::CVector3 pos = pcBox->GetEmbodiedEntity().GetOriginAnchor().Position;
        argos::CVector3 size = pcBox->GetSize();
        std::string id = it->first;
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
    for (long unsigned i = 0; i < Map_Structure::boxes.size(); i++) {
        Map_Structure::boxes[i].setBoxCorner();
        for (auto j = 0; j < 4; j++) {
            Map_Structure::points.push_back(Map_Structure::boxes[i].getVCorner(j));

            Map_Structure::hardLines.push_back(Map_Structure::boxes[i].getBoxLine(j));
        }
    }
}
int Map_Structure::getRobotById(std::string id) {
    for (long unsigned i = 0; i < Robots.size(); i++) {
        if (Robots[i].getfootBot()->GetId() == id) {
            return i;
        }
    }

    throw std::invalid_argument("Robot not found by ID");
}

void Map_Structure::setDistanceMatrix(){
    auto size = (unsigned)sqrt(Map_Structure::lines.size());

    shortestDistanceMatrix.resize(size, std::vector<float>(size));
    for(auto& line: Map_Structure::lines){
        shortestDistanceMatrix[line.Geta().getId()][line.Getb().getId()] = line.GetFloydDistance();
    }
    shortestPath.clear();
    shortestPath.resize(size, std::vector<int>(size));
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

    float inf = std::numeric_limits<float>::infinity();

    for (long unsigned k = 0; k < shortestDistanceMatrix.size(); k++) {
        for (long unsigned i = 0; i < shortestDistanceMatrix.size(); i++){
            for (long unsigned j = 0; j < shortestDistanceMatrix.size(); j++){
                float temp = shortestDistanceMatrix[i][k] + shortestDistanceMatrix[k][j];
                if (shortestDistanceMatrix[i][k]==inf || shortestDistanceMatrix[k][j]== inf)
                    temp = inf;
                if (shortestDistanceMatrix[i][k] != inf && shortestDistanceMatrix[k][j]!= inf && temp < shortestDistanceMatrix[i][j]){
                    shortestDistanceMatrix[i][j] = (shortestDistanceMatrix[i][k] + shortestDistanceMatrix[k][j]);
                    shortestPath[i][j] = shortestPath[i][k];
                }
            }
        }
    }
}

// @Todo: Could use a refactor as this code is much more complex than the pseudo-code for
// the Floyd-Warshall Algorithm.
std::vector<std::vector<float>> Map_Structure::floydShortestOfStations() {
    unsigned long amountOfStations = Map_Structure::stationIDs.size() + Map_Structure::endStationIDs.size();

    std::vector<std::vector<float>> shortestDistance(amountOfStations, std::vector<float>());
    for (long unsigned i = 0; i < amountOfStations; i++) {
        for (long unsigned j = 0; j < amountOfStations; j++) {
            shortestDistance[i].push_back(shortestDistanceMatrix[i][j]);
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
    throw std::invalid_argument("Point not found");
}

void Map_Structure::createStaticJSON() {

    //get_distance_matrix(Map_Structure::get_instance());
    //configure_static_settings_of_Uppaal_model(Map_Structure::get_instance());

    nlohmann::json jsonObj;


    jsonObj["station_delay"] = 6;
    jsonObj["waypoint_delay"] = 3;
    jsonObj["uncertainty"] = 1.1;

    //std::vector<std::pair<float,float>> xypoints;
    std::vector<std::vector<double>> xypoints;
    std::vector<int> dropIds;
    std::vector<int> contIds;

    for (Point p : Map_Structure::points) {
        if (p.getType() == pointType::endpoint) {
            dropIds.push_back(p.getId());
            std::vector<double> xy;
            xy.push_back(p.getX());
            xy.push_back(p.getY());
            xypoints.push_back(xy);
        }
        if (p.getType() == pointType::station) {
            contIds.push_back(p.getId());
            std::vector<double> xy;
            xy.push_back(p.getX());
            xy.push_back(p.getY());
            xypoints.push_back(xy);
        }
    }
    jsonObj["end_stations"] = dropIds;

    jsonObj["station_distance_matrix"] =
        floydShortestOfStations();

    jsonObj["stations"] = contIds;
    std::vector<std::vector<int>> shortestp;
    for (long unsigned i = 0; i < points.size(); i ++) {
        std::vector<int> tmp;
        for (long unsigned j = 0; j < points.size(); j ++) {
            if(i != j)
                tmp.push_back(findPath(points[i].getId(), points[j].getId())[0].getId());
            else
                tmp.push_back(points[i].getId());
        }
        shortestp.push_back(tmp);
    }
    jsonObj["shortest_paths"] = shortestp;
    std::vector<int> vias;
    for (auto & point : Map_Structure::points) {
        if (point.getType() == pointType::via) {
            vias.push_back(point.getId());
            std::vector<double> xy;
            xy.push_back(point.getX());
            xy.push_back(point.getY());
            xypoints.push_back(xy);

        }
    }
    jsonObj["vias"] = vias;
    jsonObj["coordinates"] = xypoints;
    // collect all the wayPoints in 2 dimensions
    int sizeLines = sqrt(Map_Structure::lines.size()); //@todo: Make 2-dimentional to begin with.

    std::vector<std::vector<int>> waypointsDistances(sizeLines, std::vector<int>());

    int k = -1;
    for (long unsigned i = 0; i < Map_Structure::lines.size(); i++) {
        if (i % sizeLines == 0)
            k++;
        if(Map_Structure::lines[i].GetDistance()==-1){
            waypointsDistances[k].push_back(Map_Structure::lines[i].GetDistance());
        }else waypointsDistances[k].push_back(Map_Structure::lines[i].GetDistance()/(double)VELOCITY*100);
    }
    jsonObj["waypoint_distance_matrix"] = waypointsDistances;

    // creation of waypoints json Object
    for (long unsigned i = 0; i < Map_Structure::points.size(); i++) {
        for (long unsigned j = 0; j < Map_Structure::lines.size(); j++) {
            if (Map_Structure::points[i].getId() ==
                Map_Structure::lines[j].Geta().getId()) {
                if (Map_Structure::lines[j].GetDistance() != -1)
                    Map_Structure::points[i].pushAdjID(
                        Map_Structure::lines[j].Getb().getId());
            } else
                break;
        }
    }
    std::vector<nlohmann::json> waypoints;
    for (long unsigned i = 0; i < Map_Structure::points.size(); i++) {
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
    std::cout << "Set static for robot" << std::endl;
    for (long unsigned i = 0; i < Map_Structure::Robots.size(); i++) {

        std::string tmp = folderPath +
                          Map_Structure::Robots[i].getfootBot()->GetId() + "/";
        std::cout << tmp <<std::endl;
        std::ofstream out(tmp + "static_config.json");
        out << std::setw(4) << jsonObj;

        //out << jsonObj;
    }
}
// Combines from all the points all possibleMap_Structure::lines
void Map_Structure::setAllPossibleLines() {
    for (auto& point_a : Map_Structure::points) {
        for (long unsigned j = 0; j < Map_Structure::points.size(); j++) {
            Map_Structure::lines.push_back(
                Line(&point_a, &Map_Structure::points[j]));
        }
    }
    eliminateBadLines();
}

float cross(argos::CVector3 a, argos::CVector3 b) {
    return a.GetX() * b.GetY() - a.GetY() * b.GetX();
}

float dot(argos::CVector3 a, argos::CVector3 b) {
    return a.GetX() * b.GetX() + a.GetY() * b.GetY();
}

bool intersectionInterest(argos::CVector3 m1, argos::CVector3 m2, argos::CVector3 n1, argos::CVector3 n2) {
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
    for (long unsigned i = 0; i < Map_Structure::lines.size(); i++) {
        for (long unsigned j = 0; j < Map_Structure::hardLines.size(); j++) {
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

std::vector<Point> Map_Structure::findPath(int startId, int destinationId) {
    std::vector<Point> pts;
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
void Map_Structure::initializeStations() {
    // get all the points defined in JSON file
    std::cout << folderPath +"points.json" <<std::endl;
    std::ifstream i(folderPath +"points.json");

    nlohmann::json j = nlohmann::json::parse(i);
    for (long unsigned i = 0; i < j.size(); i++) {
        if(j[i].value("x", 0.0)!= 0.0){
            Point p = Point(
                argos::CVector3(j[i].value("x", 0.0), j[i].value("y", 0.0), j[i].value("z", 0.0)),
                static_cast<pointType>(j[i].value("type", 0)), j[i].value("name", ""));

            points.push_back(p);
        }
        if (static_cast<pointType>(j[i].value("type", 0)) == pointType::station)
            stationIDs.push_back(points[i].getId());
        if (static_cast<pointType>(j[i].value("type", 0)) == pointType::endpoint)
            endStationIDs.push_back(points[i].getId());
    }
}
void Map_Structure::initializeJobs() {
    // get all the points defined in JSON file
    std::cout << folderPath +"jobs.json" <<std::endl;
    std::ifstream i(folderPath +"jobs.json");
    nlohmann::json j = nlohmann::json::parse(i);
    for (long unsigned i = 0; i < j.size(); i++) {
        jobs.push_back(j[i].value("job", std::vector<int>(0)));
    }
}
void Map_Structure::createFolderForEachRobot() {
    for (long unsigned i = 0; i < Robots.size(); i++) {

        std::string temp = folderPath + Robots[i].getfootBot()->GetId();

        if (mkdir(temp.c_str(), 0777) == -1) {
        }
    }
}
void Map_Structure::setFolderPath() {
    std::string fileLoc = argos::CSimulator::GetInstance().GetExperimentFileName();
    if(fileLoc.find("/") != std::string::npos) {
        auto pos = fileLoc.find_last_of("/");
        folderPath = fileLoc.substr(0 , pos + 1);
    }
    else {
        folderPath = "/";
    }
    std::cout << folderPath <<std::endl;
}

void Map_Structure::generateJobs() {
    //************creates random jobs in json file************
    std::random_device rd;  // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_int_distribution<> distr(2, 11);   // define the range of stations ids
    std::uniform_int_distribution<> distrEnd(2, 10); // define the range of how many stations to visit
    std::uniform_int_distribution<> endPoints(0, 1);//define end points ids
    nlohmann::json mainJsonObj;
    for (auto i = 0; i < 100; i++){
        nlohmann::json jsonObj;
        jsonObj["job_id"] = i;
        std::vector<nlohmann::json> stationsToVisit;
        int amountPickups = distrEnd(eng);
        for (auto j = 0; j < amountPickups ;j++) {
            int temp = distr(eng);
            bool check = true;
            for(auto& station: stationsToVisit){
                if(station == temp){ check = false;break;}}
            if(check)stationsToVisit.push_back(temp);
        }
        stationsToVisit.push_back(endPoints(eng));
        jsonObj["job"] = stationsToVisit;
        mainJsonObj.push_back(jsonObj);
    }
    Map_Structure sMap = Map_Structure::get_instance();
    std::ofstream out(sMap.folderPath + "/jobs.json");
    out << std::setw(4) << mainJsonObj;
}