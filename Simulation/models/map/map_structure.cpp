#include "map_structure.hpp"
#include "controllers/parsing/uppaal_model_parsing.hpp"

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
    for (auto &boxMap : tBoxMap) {
        argos::CBoxEntity *pcBox = argos::any_cast<argos::CBoxEntity *>(boxMap.second);
        argos::CVector3 pos = pcBox->GetEmbodiedEntity().GetOriginAnchor().Position;
        argos::CVector3 size = pcBox->GetSize();
        std::string id = boxMap.first;
        Box b{id, pos, size};
        switch (id[0]) {
            case 'w': // break; allow break in order to stop considering boundries of
                // arena
                walls.push_back(b);
                break;
            case 'c':
            case 'd':
            case 'o':
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

int Map_Structure::getRobotIdByName(std::string name) {
    for (long unsigned i = 0; i < Robots.size(); i++) {
        if (Robots[i].getfootBot()->GetId() == name) {
            return i;
        }
    }

    throw std::invalid_argument("Robot not found by NAME");
}

Robot Map_Structure::getRobotByName(std::string name) {
    for (auto &robot:  Robots) {
        if (robot.getfootBot()->GetId() == name) {
            return robot;
        }
    }

    throw std::invalid_argument("Robot not found by NAME");
}

// Set the fields `shortestDistanceMatrix`, which gives the length of the shortest path between all points
// and `shortestPath`, which tells the next Point to go to if one wants the shortest path.
void Map_Structure::setDistanceMatrix() {
    auto size = (unsigned) sqrt(Map_Structure::lines.size());

    shortestDistanceMatrix.resize(size, std::vector<float>(size));
    for (auto &line: Map_Structure::lines) {
        shortestDistanceMatrix[line.Geta().getId()][line.Getb().getId()] = line.GetFloydDistance();
    }
    shortestPath.clear();
    shortestPath.resize(size, std::vector<int>(size));
    std::cout << "Size of shortestDistanceMatrix: " << size << std::endl;
    for (unsigned i = 0; i < size; ++i) {
        for (unsigned j = 0; j < size; ++j) {
            shortestPath[i][j] = 0;
        }
        for (unsigned j = 0; j < size; ++j) {
            if (i != j) {
                shortestPath[i][j] = j + 1;
            }
        }
    }

    float inf = std::numeric_limits<float>::infinity();

    for (long unsigned k = 0; k < shortestDistanceMatrix.size(); k++) {
        for (long unsigned i = 0; i < shortestDistanceMatrix.size(); i++) {
            for (long unsigned j = 0; j < shortestDistanceMatrix.size(); j++) {
                float temp = shortestDistanceMatrix[i][k] + shortestDistanceMatrix[k][j];
                if (shortestDistanceMatrix[i][k] == inf || shortestDistanceMatrix[k][j] == inf)
                    temp = inf;
                if (shortestDistanceMatrix[i][k] != inf && shortestDistanceMatrix[k][j] != inf &&
                    temp < shortestDistanceMatrix[i][j]) {
                    shortestDistanceMatrix[i][j] = (shortestDistanceMatrix[i][k] + shortestDistanceMatrix[k][j]);
                    shortestPath[i][j] = shortestPath[i][k];


//                    if(i < amountOfStations && j < amountOfStations && k >= amountOfStations){
//                        shortestDistanceMatrix[i][j] = (shortestDistanceMatrix[i][k] + shortestDistanceMatrix[k][j]);
//                        shortestPath[i][j] = shortestPath[i][k];
//                    }
//                    else
//                    if(i >= amountOfStations && j >= amountOfStations){
//                        shortestDistanceMatrix[i][j] = (shortestDistanceMatrix[i][k] + shortestDistanceMatrix[k][j]);
//                        shortestPath[i][j] = shortestPath[i][k];
//                    }else {
//                        std::cout << "from "  << i << " to " << j <<"  || " << "from "  << i << " to " << k <<std::endl;
//                    }
                }
            }
        }
    }
    //DEBUG
//    for(int i = 0; i < 12; i ++){
//        for(int j = 0; j < 12; j ++){
//            if(i != j) {
//                auto result = findPath(i, j);
//                std::cout << "------Path ------" << std::endl;
//                std::cout << "from : " << getPointByID(i).getName() << std::endl;
//                for (auto &p : result) {
//                    std::cout << "ID: " << p.getName() << " -> ";
//                }
//                std::cout << "to : " << getPointByID(j).getName() << std::endl;
//            }
//        }
//    }
    //END DEBUG
}

// @Todo: Could use a refactor as this code is much more complex than the pseudo-code for
// the Floyd-Warshall Algorithm.
std::vector<std::vector<float>> Map_Structure::floydShortestOfStations() {
    std::vector<std::vector<float>> shortestDistance(amountOfStations, std::vector<float>());
    for (long unsigned i = 0; i < amountOfStations; i++) {
        for (long unsigned j = 0; j < amountOfStations; j++) {
            shortestDistance[i].push_back(shortestDistanceMatrix[i][j]);
            std::cout << "i : " << i << " | j : " << j << " | distance : " << shortestDistanceMatrix[i][j] << std::endl;
        }
    }
    return shortestDistance;
}

Point &Map_Structure::getPointByID(int id) {
    for (auto &p : points) {
        if (p.getId() == id) {
            return p;
        }
    }
    throw std::invalid_argument("Point not found");
}


// Combines from all the points all possibleMap_Structure::lines
void Map_Structure::setAllPossibleLines() {
    for (auto &point_a : Map_Structure::points) {
        for (long unsigned j = 0; j < Map_Structure::points.size(); j++) {
            Map_Structure::lines.push_back(
                    Line(&point_a, &Map_Structure::points[j]));
        }
    }
    eliminateBadLines();
}

double cross(argos::CVector3 a, argos::CVector3 b) {
    return a.GetX() * b.GetY() - a.GetY() * b.GetX();
}

double dot(argos::CVector3 a, argos::CVector3 b) {
    return a.GetX() * b.GetX() + a.GetY() * b.GetY();
}

bool areLinesParallel(const argos::CVector3 &m1, const argos::CVector3 &m2, const argos::CVector3 &n1,
                      const argos::CVector3 &n2) {
    double a1 = m2.GetY() - m1.GetY();
    double b1 = m1.GetX() - m2.GetX();

    double a2 = n2.GetY() - n1.GetY();
    double b2 = n2.GetX() - n1.GetX();

    return 0 == a1 * b2 - a2 * b1;
}

bool intersectionInterest(argos::CVector3 m1, argos::CVector3 m2, argos::CVector3 n1, argos::CVector3 n2) {
    //If lines are parallel, stop the check and return false
    if (areLinesParallel(m1, m2, n1, n2)) return false;

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


bool checkIfPointIsInShapeHelper(Box &box, Line &line, Line &vLine, bool isA) {
    if (box.isPointInShape(isA ? line.Geta() : line.Getb())) {
        //If such point is connected to one of the boxes virtual corners - eliminate
//        if (box.isPointPartOfTheBox(isA ? line.Getb() : line.Geta())) {
//            line.setFailureline();
//            return true;
//        }
        //Once inside the box check if we are comparing with the closest line, if so skip
        Line l = box.getClosestLineToAPoint(isA ? line.Geta() : line.Getb());
        if (vLine == l)
            return true;
    }
    return false;
}

//threshold used for determining if the point is on the line or not
const float threshold = 0.2f;

bool cross2(Point &p, Point &q, Point &r) {
    double dxc = r.getX() - p.getX();
    double dyc = r.getY() - p.getY();

    double dxl = q.getX() - p.getX();
    double dyl = q.getY() - p.getY();
    double cross = dxc * dyl - dyc * dxl;
    if (abs(cross) > threshold) return false;

    if (abs(dxl) >= abs(dyl))
        return dxl > 0 ?
               p.getX() <= r.getX() && r.getX() <= q.getX() :
               q.getX() <= r.getX() && r.getX() <= p.getX();
    else
        return dyl > 0 ?
               p.getY() <= r.getY() && r.getY() <= q.getY() :
               q.getY() <= r.getY() && r.getY() <= r.getY();
}

void Map_Structure::compareWithVirtualLines(Line &line) {
    for (auto &box : boxes) {
        for (auto &vLines : box.getVirtualLines()) {
            //check if a point is inside the virtual box
            if (checkIfPointIsInShapeHelper(box, line, vLines, true) ||
                checkIfPointIsInShapeHelper(box, line, vLines, false)) {
                continue;
            }
            //if normal line intersects with any of the virtual lines, we mark it as incorrect line
            if (!(line == vLines)) {
                if (intersectionInterest(line.Geta(), line.Getb(), vLines.Geta(), vLines.Getb())) {
                    line.setFailureline();
                    break;
                }
            }
        }
    }
}

//Loop through all points and if they do not belong to the line checks if
//any of the points fall under the line segment, if so sets such line as a "bad" line
void Map_Structure::doesLineCrossPoint(Line &line) {
    for (auto &point : points) {
        if (point != line.Getb() && point != line.Geta())
            if (cross2(line.Geta(), line.Getb(), point)) {
                std::cout << line.Geta().getName() << " with " << line.Getb().getName() << " cross " << point.getName()
                          << std::endl;
                line.setFailureline();
                break;
            }
    }
}

// Functions eliminates all theMap_Structure::lines which have intersection
void Map_Structure::eliminateBadLines() {
    std::vector<int> adjIDs;
    for (long unsigned i = 0; i < Map_Structure::lines.size(); i++) {
        for (auto &hardLine : hardLines) {
            if (intersectionInterest(Map_Structure::lines[i].Geta(),
                                     Map_Structure::lines[i].Getb(),
                                     hardLine.Geta(), hardLine.Getb())) {
                Map_Structure::lines[i].setFailureline();
                break;
            }
        }
        //Checks if any of the lines intersect with the virtual lines
        if (Map_Structure::lines[i].GetDistance() > 0)
            compareWithVirtualLines(Map_Structure::lines[i]);
        //Check if any of the lines overlap with other line
        if (Map_Structure::lines[i].GetDistance() > 0)
            doesLineCrossPoint(Map_Structure::lines[i]);

        //This part of the code responsible for adjID's
        //If one decides that they are not needed can be freely removed
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
        //End of adjID's
    }
}

std::vector<Point> Map_Structure::findPath(int startId, int destinationId) {
    std::vector<Point> pts;
    if (startId == destinationId) {
        return pts;
    }
    auto u = startId + 1;
    auto v = destinationId + 1;
    do {
        u = shortestPath[u - 1][v - 1];
        pts.push_back(getPointByID(u - 1));
    } while (u != v);
    return pts;
}

void Map_Structure::initializeStations() {
    // get all the points defined in JSON file
    std::cout << folderPath + "points.json" << std::endl;
    std::ifstream i(folderPath + "points.json");

    nlohmann::json j = nlohmann::json::parse(i);
    for (long unsigned i = 0; i < j.size(); i++) {
        if (j[i].value("x", 0.0) != 0.0) {
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
    amountOfStations = stationIDs.size() + endStationIDs.size();
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
    if (fileLoc.find("/") != std::string::npos) {
        auto pos = fileLoc.find_last_of("/");
        folderPath = fileLoc.substr(0, pos + 1);
    } else {
        folderPath = "/";
    }
    std::cout << folderPath << std::endl;
}

int Map_Structure::getAmountOfStations() { return (int) amountOfStations; }