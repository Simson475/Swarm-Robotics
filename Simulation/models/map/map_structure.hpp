#ifndef MAP_STRUCTURE
#define MAP_STRUCTURE

#include "box.hpp"
#include "line.hpp"
#include "point.hpp"
#include "models/robot/robot.hpp"

#include "nlohmann/json.hpp"
#include "argos3/core/simulator/loop_functions.h"
#include "argos3/plugins/robots/foot-bot/simulator/footbot_entity.h"
#include "argos3/plugins/simulator/entities/box_entity.h"
#include "argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h"


class Map_Structure {
public:
    std::string folderPath;
    //storage of elements in the map
    std::vector<Point> points;
    std::vector<Line> lines;
    std::vector<Box> boxes;
    std::vector<Line> hardLines;
    std::vector<int> stationIDs;
    std::vector<int> endStationIDs;
    std::vector<int> waypointsIDs;
    std::vector<Robot> Robots;
    std::vector<std::vector<int>> jobs;
    //shortest paths between each point
    std::vector<std::vector<int>> shortestPath;
    //shortest distances between each station
    std::vector<std::vector<float>> shortestDistances;
    //counters to figure how often uppaal was too slow
    int timesUppaalFailed = 0;
    int totalTries = 0;


    const std::vector<Line>& get_lines() const {
        return lines;
    }

    //ensurance that the class is created only once
    static Map_Structure &get_instance() {
        static Map_Structure instance;
        return instance;
    }

    //sets path of the folder where one can find trajectory and points
    void setFolderPath();

    // finds a robot by an ID
    int getRobotById(std::string id);

    //collects all waypoints from the map including station/ end point/ via/ start locations
    void collectAllWayPoints();

    //creates a static_config.json file with all the relevant data
    void createStaticJSON();

    //usage of Floyd-Warshall Algorithm for shortest paths between each via
    std::vector<std::vector<float>> floydShortestOfStations() ;

    // function which sets all possible lines between all the points
    void setAllPossibleLines();

    // functions eliminates all lines which have intersection with any of the hard lines
    void eliminateBadLines();

    Point& getPointByID(int id);

    //finds the shortest path of vias from startId to destinationId
    std::vector<Point> findPath(int startId, int destinationId);

    //collects all the stations from experiment/scene2/points.json
    void initializeStations();

    //collects all the stations from experiment/scene2/points.json
    void initializeJobs();

    //creates for each simulated robot a folder, where one stores it's config files
    void createFolderForEachRobot();

private:
//private constructor ensuring that only one instance is being created of the class
    Map_Structure() = default;
};
#endif