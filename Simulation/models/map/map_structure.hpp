#ifndef MAP_STRUCTURE
#define MAP_STRUCTURE

#include "box.hpp"
#include "line.hpp"
#include "point.hpp"

#include "nlohmann/json.hpp"
#include "argos3/core/simulator/loop_functions.h"
#include "argos3/plugins/robots/foot-bot/simulator/footbot_entity.h"
#include "argos3/plugins/simulator/entities/box_entity.h"

#include <random>
#include <limits>

class Map_Structure {
private:
    // Shortest distance between each point
    std::vector<std::vector<float>> shortestDistanceMatrix{};
    std::vector<std::vector<float>> realShortestDistanceMatrix{};
    //amount of endStations and normal stations on the map
    uint amountOfStations;
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
    //shortest paths between each point
    std::vector<std::vector<int>> shortestPath;
    std::vector<std::vector<int>> realShortestPath;


    //ensures that the class is created only once
    static Map_Structure &get_instance() {
        static Map_Structure instance;
        return instance;
    }

    //sets path of the folder where one can find trajectory and points
    void setFolderPath();

    // Get shortest Distances
    const std::vector<std::vector<float>> &getShortestDistanceMatrix() const {
        return shortestDistanceMatrix;
    };

    const std::vector<std::vector<float>> &getRealShortestDistanceMatrix() const {
        return realShortestDistanceMatrix;
    };

    //collects all waypoints from the map including station/ end point/ via/ start locations
    void collectAllWayPoints();


    //usage of Floyd-Warshall Algorithm for shortest paths between each via
    std::vector<std::vector<float>> floydShortestOfStations();

    // function which sets all possible lines between all the points
    void setAllPossibleLines();

    // eliminates a line if it crosses any of the points
    bool doesLineCrossPoint(Line &line);

    // functions eliminates all lines which have intersection with any of the hard lines
    void eliminateBadLines();

    //Helper function for eliminateBadLines
    bool intersectWithVirtualLines(Line &line);

    Point &getPointByID(int id);

    //finds the shortest path of vias from startId to destinationId
    std::vector<Point> findPath(int startId, int destinationId);

    //collects all the stations from experiment/scene2/points.json
    void initializeStations();

    // Sets up the distance matrix for all paths
    void setDistanceMatrix();
    void setRealDistanceMatrix();

    //Removes points which are too close to each other
    void eliminateBadPoints();

    // For occupying a point
    bool isPointAvailable(int id);

    void setPointAsOccupied(int id);

    void setPointAsAvailable(int id);

    int getAmountOfStations();

    int getIdOfFirstStartStation();
    int getIdOfLastStartStation();

private:
//private constructor ensuring that only one instance is being created of the class
    Map_Structure() = default;
};

#endif