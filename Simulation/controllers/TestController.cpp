#include "TestController.hpp"
#include <iostream>
#

std::vector<int> TestController::constructStationPlan(){
    // std::cout << "\n\n\n Testing station plan \n\n\n";
    return SingleThreadBotGreedy::constructStationPlan();
}
std::vector<int> TestController::constructWaypointPlan(){
    if ( !receivedWaypointPlan.empty()){
        std::vector<int> tempPlan = receivedWaypointPlan;
        receivedWaypointPlan.erase(receivedWaypointPlan.begin()); //TODO make pretty
        return tempPlan;
    }
    auto highlevel = HighLevelCBS::get_instance();
    highlevel.findSolution();
    // std::cout << "\n\n\n\n\n";
    // std::cout << "test";
    // for(size_t i=0; i < receivedWaypointPlan.size();i++){
    //    std::cout << receivedWaypointPlan[i] << " ";
    // }
    // std::cout << "\n\n\n\n\n";
    return receivedWaypointPlan;
}

std::vector<Point> TestController::findOptimalPath(){
    return sMap.findPath(lastLocation, stationPlan.front());
}

void TestController::specialInit(){
    subtype = "CBS";
}

REGISTER_CONTROLLER(TestController, "TestController")