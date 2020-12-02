#ifndef ROBOT
#define ROBOT

#include "models/map/point.hpp"

#include "nlohmann/json.hpp"
#include "argos3/core/simulator/loop_functions.h"
#include "argos3/plugins/robots/foot-bot/simulator/footbot_entity.h"
#include "argos3/plugins/simulator/entities/box_entity.h"
#include "argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h"

#include <math.h>

#define stationDelay 60

struct TimeValuePair {
    double time;
    int value;
};

struct SimulationTrace {
    int number;
    std::vector<TimeValuePair> values;
};
struct SimulationExpression {
    std::string name;
    std::vector<SimulationTrace> runs;
};
enum class Status {
    available, // if the robot does not have a job.
    requestStations,
    waitStations,
    requestWaypoints,
    waitWaypoints,
    occupied
};

struct timeResult {std::string id;double distance; int stationsPassed; std::vector<Point> waypointsToPass;
    argos::CVector3 currPosition; bool found; std::vector<Point> allPassedPoints;};

class Robot {
private:
    std::vector<Point> job;
    std::vector<SimulationExpression> stationPath;
    std::vector<SimulationExpression> waypointPath;
    std::vector<Point> visitedStations;
    std::vector<Point> visitedWaypoints;
    argos::CFootBotEntity *footBot;
    // Point nextWaypoint;
    Point* initialLocation;
    std::vector<Point> remainingStations;
    std::vector<Point> remainingWaypoints;
    std::vector<int> stationPlan;
    Status status;
    double etaNextStation;
    int stopWatch = -1;
    Point* latestPoint;
    Point* currTarget;
    int previousLoc;
    std::vector<timeResult*> otherRobotsInf;

    void clearRobotInf();

public:
    Robot(argos::CFootBotEntity *footBot, Point *initialLoc);
    Robot& operator=(const Robot& that);
    ~Robot() = default;
    Point& getInitialLoc();
    void increment(int i);
    void setCurrStationTarget();
    Point* getCurrentTarget() const {return currTarget;}
    void addSinglePickUp(Point pickup);
    void removeFirstStation();
    bool hasJob();
    void cleanJob();
    Point *getNextWayPoint();
    void changeStatus(Status stat);
    double getEta();
    int getWatch() { return stopWatch; }
    bool atPoint() const { return stopWatch != -1;}
    std::vector<Point> getJob() { return job; }
    std::vector<int> getJobByIds();
    std::vector<Point> getRemainingStations() const { return remainingStations; }
    void clearStations(){remainingStations.clear();}
    void clearWaypoints(){remainingWaypoints.clear();}
    std::vector<Point> getRemainingWaypoints() { return remainingWaypoints; }
    argos::CFootBotEntity *getfootBot() const { return footBot; }
    Status getStatus() const { return status; }
    bool contains(int id, std::vector<Point>& points);
    std::vector<Point> setRemainingStations(std::vector<Point> allPoints);
    std::vector<Point> setRemainingWaypoints(std::vector<Point> &allPoints);
    void converJSONStation(std::string robotId, std::string choice);
    std::string createDynamicJson(std::vector<Robot> &robots, Robot &robot, bool stations);
    void sortJob(const std::vector<std::vector<float>>& shortestDistances);//Sorts the points according to the shortest distance
    void addWaypoints(std::vector<Point> path);
    void updateCurrent(Point* target);
    // Returns the most recent point the robot had been at.
    const Point& getLatestPoint() const {return *latestPoint;}
    timeResult* getEtaNextRobot(Robot r, double timeToDelay);
    timeResult* getEtaHelper(std::string id, std::vector<Point> waypoints, argos::CVector3 currPosition, double timeToDelay, double temp, std::vector<Point>& allWaypoints);
    int getPreviousLoc(){return previousLoc;}
    void setPreviousLoc(int p){previousLoc = p;}
    std::vector<timeResult*> getOtherRobotsEstimates(){return otherRobotsInf;}
    argos::CVector3 getPosition(argos::CVector3 start, argos::CVector3 end, double distance);
    std::string getName() const;

    bool operator==(const Robot &r) const {
        return getfootBot() == r.getfootBot();
    }

    bool operator!=(const Robot &r) const {
        return getfootBot() != r.getfootBot();
    }

    //************************* Functionality to plan in interface
    void setStationPlan(std::vector<int>);
    std::vector<int> getStationPlan();
};
#endif