#include "TestController.hpp"
#include <iostream>

std::vector<int> TestController::constructStationPlan(){
    // std::cout << "Testing station plan";
    return SingleThreadBotGreedy::constructStationPlan();
}
std::vector<int> TestController::constructWaypointPlan(){
    //std::cout << "Testing waypoint plan";

    if ( !receivedWaypointPlan.empty()){
        receivedWaypointPlan.erase(receivedWaypointPlan.begin()); //TODO make pretty
        return receivedWaypointPlan;
    }

    // Ensure that every other bot has gotten a station plan at this point
    for (auto &bot : otherBots){
        if (bot.get().getStationPlan().empty()){
            return {};// If not all bots have a station plan, just go idle
        }
    }
    auto highlevel = HighLevelCBS::get_instance();
    highlevel.findSolution();
    std::cout << "WE GO HERE";

    
    return receivedWaypointPlan;
}

std::vector<Point> TestController::findOptimalPath(){
    return sMap.findPath(lastLocation, stationPlan.front());
}

REGISTER_CONTROLLER(TestController, "TestController")