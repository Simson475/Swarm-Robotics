#include "TestController.hpp"
#include <iostream>

std::vector<int> TestController::constructStationPlan(){
    std::cout << "Testing station plan";
    return SingleThreadBotGreedy::constructStationPlan();
}
std::vector<int> TestController::constructWaypointPlan(){
    std::cout << "Testing waypoint plan";
    //return SingleThreadBotGreedy::constructWaypointPlan();
    if ( !receivedWaypointPlan.empty()){
        receivedWaypointPlan.erase(receivedWaypointPlan.begin());
        return receivedWaypointPlan;
    }
    auto highlevel = HighLevelCBS::get_instance();
    highlevel.findSolution();
    
    return receivedWaypointPlan;
}

std::vector<Point> TestController::findOptimalPath(){
    //return sMap.findPath(0, stationPlan.front());
    return sMap.findPath(1, 5);
}

REGISTER_CONTROLLER(TestController, "TestController")