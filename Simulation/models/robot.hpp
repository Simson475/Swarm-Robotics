#ifndef ROBOT
#define ROBOT

#include "nlohmann/json.hpp"
#include "point.hpp"
#include <math.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

//using namespace std;
using namespace argos;
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
enum Status {
    available,
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
    CFootBotEntity *footBot;
    // Point nextWaypoint;
    Point* initialLocation;
    double etaNextWayPoint;
    std::vector<Point> remainingStations;
    std::vector<Point> remainingWaypoints;
    Status status;
    double etaNextStation;
    int stopWatch = -1;
    Point* currentPositionId;
    Point* currTarget;
    int previousLoc;
    std::vector<timeResult*> otherRobotsInf;

public:
    Robot(CFootBotEntity *footBot, Point *initialLoc);
    Robot& operator=(const Robot& that);
    ~Robot(){}
    Point& getInitialLoc();
    Point getNextStation();
    void increment(int i);
    void setEta(double time);
    void setCurrStationTarget();
    Point* getCurrentTarget(){return currTarget;}
    void setJob(std::vector<Point> &jobs);
    void addSinglePickUp(Point pickup);
    void removeFirstStation();
    void removeFirstWaypoint();
    void cleanJob();
    Point *getNextWayPoint();
    void changeStatus(Status stat);
    double getEta();
    int getWatch() { return stopWatch; }
    std::vector<Point> getJob() { return job; }
    std::vector<Point> getRemainingStations() { return remainingStations; }
    void clearStations(){remainingStations.clear();}
    void clearWaypoints(){remainingWaypoints.clear();}
    std::vector<Point> getRemainingWaypoints() { return remainingWaypoints; }
    CFootBotEntity *getfootBot() const { return footBot; }
    Status getStatus() { return status; }
    bool contains(int id, std::vector<Point>& points);
    std::vector<Point> setRemainingStations(std::vector<Point> allPoints);
    std::vector<Point> setRemainingWaypoints(std::vector<Point> &allPoints);
    void converJSONStation(std::string robotId, std::string choice);
    void addEndPoint();
    std::string createDynamicJson(std::vector<Robot> &robots, Robot &robot, bool stations);
    void createWaypointQ(std::vector<Robot> &robots, std::vector<Point> &points, int n);
    void sortJob(std::vector<std::vector<float>> shortestDistances);//Sorts the points according to the shortest distance
    void addWaypoints(std::vector<Point> path);
    void updateCurrent(Point* target);
    Point& getCurrentID(){return *currentPositionId;}
    timeResult* getEtaNextRobot(Robot r, double timeToDelay);
    timeResult* getEtaHelper(std::string id, std::vector<Point> waypoints, argos::CVector3 currPosition, double timeToDelay, double temp, std::vector<Point>& allWaypoints);
    int getPreviousLoc(){return previousLoc;}
    void setPreviousLoc(int p){previousLoc = p;}
    std::vector<timeResult*> getOtherRobotsEstimates(){return otherRobotsInf;}
    argos::CVector3 getPosition(CVector3 start, CVector3 end, double distance);
    //friend bool operator== (const Robot &r1, const Robot &r2);

    bool operator==(const Robot &r) const {
        return getfootBot() == r.getfootBot();
    }

};
#endif