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

    bool include_corners = true;
    argos::TConfigurationNode& t_node = argos::CSimulator::GetInstance().GetConfigurationRoot();
    argos::TConfigurationNode& params = argos::GetNode(t_node, "custom");
    argos::GetNodeAttributeOrDefault(params, "include_corners", include_corners, include_corners);

    for (long unsigned i = 0; i < Map_Structure::boxes.size(); i++) {
        Map_Structure::boxes[i].setBoxCorner();
        for (auto j = 0; j < 4; j++) {
            if (include_corners)
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

    float inf = std::numeric_limits<float>::infinity();
    //We start with k being amountOfStations in order to avoid having
    //a station in between two points being threated as a via
    for (long unsigned k = amountOfStations; k < shortestDistanceMatrix.size(); k++) {
        for (long unsigned i = 0; i < shortestDistanceMatrix.size(); i++) {
            for (long unsigned j = 0; j < shortestDistanceMatrix.size(); j++) {
                float temp = shortestDistanceMatrix[i][k] + shortestDistanceMatrix[k][j];
                if (shortestDistanceMatrix[i][k] != inf && shortestDistanceMatrix[k][j] != inf &&
                    temp < shortestDistanceMatrix[i][j]) {
                    shortestDistanceMatrix[i][j] = (shortestDistanceMatrix[i][k] + shortestDistanceMatrix[k][j]);
                }
            }
        }
    }
}

void Map_Structure::setRealDistanceMatrix() {
    auto size = (unsigned) sqrt(Map_Structure::lines.size());
    argos::LOG << "Size: " << size << std::endl;
    argos::LOG << "point: " << points.size() << std::endl;
    realShortestPath.clear();
    realShortestPath.resize(size, std::vector<int>(size));
    for (unsigned i = 0; i < size; ++i) {
        for (unsigned j = 0; j < size; ++j) {
            if (i != j)
                realShortestPath[i][j] = -1;
            else
                realShortestPath[i][j] = i;
        }
    }


    realShortestDistanceMatrix.resize(size, std::vector<float>(size));
    for (auto &line: Map_Structure::lines) {
        realShortestDistanceMatrix[line.Geta().getId()][line.Getb().getId()] = line.GetFloydDistance();

        if (line.GetFloydDistance() != std::numeric_limits<float>::infinity()){
            realShortestPath[line.Geta().getId()][line.Getb().getId()] = line.Getb().getId();
            realShortestPath[line.Getb().getId()][line.Geta().getId()] = line.Geta().getId();
        }
    }


    for (long unsigned k = 0; k < realShortestDistanceMatrix.size(); k++) {
        for (long unsigned i = 0; i < realShortestDistanceMatrix.size(); i++) {
            for (long unsigned j = 0; j < realShortestDistanceMatrix.size(); j++) {
                if (realShortestDistanceMatrix[i][j] > realShortestDistanceMatrix[i][k] + realShortestDistanceMatrix[k][j]) {
                    realShortestDistanceMatrix[i][j] = realShortestDistanceMatrix[i][k] + realShortestDistanceMatrix[k][j];
                    realShortestPath[i][j] = realShortestPath[i][k];
                }
            }
        }
    }
}

// @Todo: Could use a refactor as this code is much more complex than the pseudo-code for
// the Floyd-Warshall Algorithm.
std::vector<std::vector<float>> Map_Structure::floydShortestOfStations() {
    std::vector<std::vector<float>> shortestDistance(amountOfStations, std::vector<float>());
    for (long unsigned i = 0; i < amountOfStations; i++) {
        for (long unsigned j = 0; j < amountOfStations; j++) {
            shortestDistance[i].push_back(shortestDistanceMatrix[i][j]);
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
    //exit(-1);
    throw std::invalid_argument("Point not found");
}

bool Map_Structure::isPointAvailable(int id) {
    return !getPointByID(id).isOccupied();
}

void Map_Structure::setPointAsOccupied(int id) {
    getPointByID(id).setAsOccupied();
}

void Map_Structure::setPointAsAvailable(int id) {
    getPointByID(id).setAsAvailable();
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

double cross(const Point &a, const Point &b) {
    return a.GetX() * b.GetY() - a.GetY() * b.GetX();
}

double dot(const Point &a, const Point &b) {
    return a.GetX() * b.GetX() + a.GetY() * b.GetY();
}

const double parallelThreshold = 0.1;

bool areLinesParallel(const Point &m1, const Point &m2, const Point &n1, const Point &n2) {
    double a1 = m2.GetY() - m1.GetY();
    double b1 = m1.GetX() - m2.GetX();

    double a2 = n2.GetY() - n1.GetY();
    double b2 = n2.GetX() - n1.GetX();

    return parallelThreshold > abs(a1 * b2 - a2 * b1);
}

//Function responsible to check if two lines intersect
bool intersectionInterest(const Point &m1, const Point &m2, const Point &n1, const Point &n2) {
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

bool checkIfPointIsInShapeHelper(Box &box, Point &p, Line &vLine) {
    if (box.isPointInShape(p)) {
        //Once inside the box check if we are comparing with the closest line, if so skip
        Line l = box.getClosestLineToAPoint(p);
        if (vLine == l)
            return true;
    }
    return false;
}

//@todo: Only check other waypoints! And see if the point is in the range of the x and y values with a slight offset.
// If yes, calculate the distance to the point.
bool doesLineCrossPointHelper(Line &l, const Point &r) {
    double offset = 0.05;

    if(r.getName().at(0) == 'S' ||
        l.Geta().getName().at(0) == 'S' ||
        l.Getb().getName().at(0) == 'S') {
        return false;
    }

    double x_high = l.Geta().getX() >= l.Getb().getX() ? l.Geta().getX() : l.Getb().getX();
    double x_low = l.Geta().getX() < l.Getb().getX() ? l.Geta().getX() : l.Getb().getX();

    double y_high = l.Geta().getY() >= l.Getb().getY() ? l.Geta().getY() : l.Getb().getY();
    double y_low = l.Geta().getY() < l.Getb().getY() ? l.Geta().getY() : l.Getb().getY();

    if (x_high + offset >= r.getX() &&
        r.getX() >= x_low - offset &&
        y_high + offset >= r.getY() &&
        r.getY() >= y_low - offset) {
            double dist = abs((l.Getb().getX() - l.Geta().getX()) * (l.Geta().getY() - r.getY()) - (l.Geta().getX() - r.getX()) * (l.Getb().getY() - l.Geta().getY())) / sqrt(pow((l.Getb().getX() - l.Geta().getX() ),2) + pow((l.Getb().getY() - l.Geta().getY() ),2));
            return dist <= offset;
    }
    else {
        return false;
    }
}

bool Map_Structure::intersectWithVirtualLines(Line &line) {
    for (auto &box : boxes) {
        for (auto &vLines : box.getVirtualLines()) {
            //check if a point is inside the virtual box
            if (checkIfPointIsInShapeHelper(box, line.Geta(), vLines) ||
                checkIfPointIsInShapeHelper(box, line.Getb(), vLines)) {
                continue;
            }
            //if normal line intersects with any of the virtual lines, we mark it as incorrect line
            if (!(line == vLines)) {
                if (intersectionInterest(line.Geta(), line.Getb(), vLines.Geta(), vLines.Getb())) {
                    return true;
                }
            }
        }
    }
    return false;
}

//Loop through all points and if they do not belong to the line checks if
//any of the points fall under the line segment, if so sets such line as a "bad" line
bool Map_Structure::doesLineCrossPoint(Line &line) {
    for (auto &point : points) {
        if (point != line.Getb() && point != line.Geta())
            if (doesLineCrossPointHelper(line, point)) {
                if(line.Geta().getId() == 99 && line.Getb().getId() == 102 )
                    argos::LOG << "Error point: " << point.getId() << std::endl;
                return true;
            }
    }
    return false;
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
            if (intersectWithVirtualLines(Map_Structure::lines[i]))
                Map_Structure::lines[i].setFailureline();
        //Check if any of the lines overlap with other line
        if (Map_Structure::lines[i].GetDistance() > 0)
            if (doesLineCrossPoint(Map_Structure::lines[i]))
                Map_Structure::lines[i].setFailureline();
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
    auto u = startId;
    auto v = destinationId;
    do {
        u = realShortestPath[u][v];
        //argos::LOG << "ID: " <<
        pts.push_back(getPointByID(u));
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

//Helper function for elimination of bad points
//When point is removed the ID's must be aligned
void fixPointsIDs(std::vector<Point> &points) {
    int id = 0;
    for (auto &p : points) {
        if (p.getId() == id)
            id++;
        else {
            p.setID(id);
            id++;
        }
    }
}

const float thresholdPointsBeingTooClose = 0.2;

void Map_Structure::eliminateBadPoints() {
    for (auto itr = points.begin(); itr < points.end(); itr++) {
        for (auto itr2 = points.begin(); itr2 < points.end(); itr2++) {
            if (itr != itr2 && itr->getDistance(*itr2) < thresholdPointsBeingTooClose) {
                itr->adjustPointToMid(*itr2);
                points.erase(itr2);
                fixPointsIDs(points);
                break;
            }
        }
    }
}