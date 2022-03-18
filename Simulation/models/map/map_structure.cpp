#include "map_structure.hpp"
#include "controllers/parsing/uppaal_model_parsing.hpp"
#include <exception>
#include <iterator>
#include <math.h>

void Map_Structure::collectAllWayPoints() {
    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto it = tBotMap.begin(); it != tBotMap.end();
         ++it) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(it->second);
        argos::CVector3 pos = pcBot->GetEmbodiedEntity().GetOriginAnchor().Position;
        
        Point *p = new Point(pos, pointType::via, "S." + pcBot->GetId());
        points.push_back(*p);
        waypointsIDs.push_back(points.back().getId());
    }

    std::vector<Box> walls;
    argos::CSpace::TMapPerType &tBoxMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("box");
    bool isCBSmap = false;
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
                b.shouldSetWaypoints = true;
                Map_Structure::boxes.push_back(b);
                break;
            case 'n': 
                b.shouldSetWaypoints = false;
                isCBSmap = true;
                Map_Structure::boxes.push_back(b);
                break;
        }
    }

    bool include_corners = true;
    try {
        argos::TConfigurationNode &t_node = argos::CSimulator::GetInstance().GetConfigurationRoot();
        argos::TConfigurationNode &params = argos::GetNode(t_node, "custom");
        argos::GetNodeAttributeOrDefault(params, "include_corners", include_corners, include_corners);
    }
    catch (argos::CARGoSException& e){}
    
    std::tuple<float,float> topRight{}; //needed for corners in edge of arena
    std::tuple<float,float> bottomLeft{};  //needed for corners in edge of arena
    auto boxlist =  Map_Structure::boxes;
    for (long unsigned i = 0; i < boxlist.size(); i++) {
        Map_Structure::boxes[i].setBoxCorner();
        if(Map_Structure::boxes[i].shouldSetWaypoints && include_corners && isCBSmap){
            auto x = Map_Structure::boxes[i].x;
            auto y = Map_Structure::boxes[i].y;
            if(x > std::get<0>(topRight)) std::get<0>(topRight) = x;
            if(y > std::get<1>(topRight)) std::get<1>(topRight) = y;
            if(x < std::get<0>(bottomLeft)) std::get<0>(bottomLeft) = x; 
            if(y < std::get<1>(bottomLeft)) std::get<1>(bottomLeft) = y;
            
            Map_Structure::points.push_back(Map_Structure::boxes[i].getVCorner(1));
            Map_Structure::points.push_back(Map_Structure::boxes[i].getVCorner(3));
        }
        for (auto j = 0; j < 4; j++) {
            Map_Structure::hardLines.push_back(Map_Structure::boxes[i].getBoxLine(j));
            if(!isCBSmap) Map_Structure::points.push_back(Map_Structure::boxes[i].getVCorner(j));

        }            
        
    }
    
    if (include_corners && isCBSmap){ //handle corner cases
        Box max = find_if(boxlist.begin(), boxlist.end(), //Find box with higest x and y
                [&topRight](const Box& obj)
                {return obj.x == std::get<0>(topRight)
                && obj.y == std::get<1>(topRight)
                && obj.shouldSetWaypoints;})[0]; 

        Box min = find_if(boxlist.begin(), boxlist.end(), //Find box with lowest x and y
                [&bottomLeft](const Box& obj)
                {return obj.x == std::get<0>(bottomLeft)
                && obj.y == std::get<1>(bottomLeft)
                && obj.shouldSetWaypoints;})[0]; 

        Map_Structure::points.push_back(max.getVCorner(0));
        Map_Structure::points.push_back(min.getVCorner(2));
    }   
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

const double parallelThreshold = 0.00001;

bool areLinesParallel(const Point &m1, const Point &m2, const Point &n1, const Point &n2) {

    if (m1.getX() - m2.getX() == 0 || n1.getX() - n2.getX() == 0){
        if(m1.getX() - m2.getX() == 0 && n1.getX() - n2.getX() == 0) {
            return true;
        }
        if(m1.getX() - m2.getX() == 0 ){
            double angle_degrees =  atan((n2.getY() - n1.getY())/ (n2.getX() - n1.getX()));
            return parallelThreshold > abs(90 - angle_degrees) || parallelThreshold > abs(270 - angle_degrees);
        }
        if(n1.getX() - n2.getX() == 0 ){
            double angle_degrees =  atan((m2.getY() - m1.getY())/ (m2.getX() - m1.getX()));
            return parallelThreshold > abs(90 - angle_degrees) || parallelThreshold > abs(270 - angle_degrees);
        }
    }

    double m_slope = (m1.getY() - m2.getY()) / (m1.getX() - m2.getX());
    double n_slope = (n1.getY() - n2.getY()) / (n1.getX() - n2.getX());

    return parallelThreshold > abs(m_slope - n_slope);
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    double val = (q.getY() - p.getY()) * (r.getX() - q.getX()) -
              (q.getX() - p.getX()) * (r.getY() - q.getY());

    if (val == 0) return 0;  // collinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}


//Function responsible to check if two lines intersect
bool intersectionInterest(const Point &p1, const Point &p2, const Point &q1, const Point &q2) {
    //If lines are parallel, stop the check and return false

    if (areLinesParallel(p1, p2, q1, q2))
        return false;

    // Find the four orientations needed for general case
    int o1 = orientation(p1, p2, q1);
    int o2 = orientation(p1, p2, q2);
    int o3 = orientation(q1, q2, p1);
    int o4 = orientation(q1, q2, p2);

    // General case
    if (o1 != o2 && o3 != o4) {
        if(p1.getId() == 14 && p2.getId() == 57) {
            std::cout << "intersects with (" << q1.getX() << ";" << q1.getY() << ") ";
            std::cout << "and  (" << q2.getX() << ";" << q2.getY() << ") " << std::endl;
            std::cout << "14: (" << p1.getX() << ";" << p1.getY() << ")" <<std::endl;
            std::cout << "57: (" << p2.getX() << ";" << p2.getY() << ")" <<std::endl;
        }
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

        for (auto &virtualLine : box.getVirtualLines()) {
            //if normal line intersects with any of the virtual lines, we mark it as incorrect line
            if (!(line == virtualLine) && !doesLineCrossPointHelper(virtualLine, line.Geta()) && !doesLineCrossPointHelper(virtualLine, line.Getb())) {
                if (intersectionInterest(line.Geta(), line.Getb(), virtualLine.Geta(), virtualLine.Getb())) {
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
    for (auto& line : Map_Structure::lines) {
        for (auto &hardLine : hardLines) {
            if (intersectionInterest(line.Geta(),
                                     line.Getb(),
                                     hardLine.Geta(), hardLine.Getb())) {
                line.setFailureline();
                break;
            }
        }

        //Checks if any of the lines intersect with the virtual lines
        if (line.GetDistance() > 0)
            if (intersectWithVirtualLines(line))
                line.setFailureline();

        //Check if any of the lines overlap with other line
        if (line.GetDistance() > 0)
            if (doesLineCrossPoint(line))
                line.setFailureline();
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

int Map_Structure::getIdOfFirstStartStation(){
    for(auto id : waypointsIDs){
        if(points[id].getName().find("S.") != std::string::npos)
            return id;
    }
    throw std::runtime_error("Could not find a start location");
}

int Map_Structure::getIdOfLastStartStation(){
    for(auto it = waypointsIDs.rbegin(); it != waypointsIDs.rend(); it++){
        if(points[*it].getName().find("S.") != std::string::npos)
            return *it;
    }
    throw std::runtime_error("Could not find a start location");
}