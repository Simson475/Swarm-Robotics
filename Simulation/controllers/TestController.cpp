#include "TestController.hpp"
#include <iostream>

std::vector<int> TestController::constructStationPlan(){
    // std::cout << "Testing station plan";
    return SingleThreadBotGreedy::constructStationPlan();
}
std::vector<int> TestController::constructWaypointPlan(){

    if ( !receivedWaypointPlan.empty()){
        std::vector<int> tempPlan = receivedWaypointPlan;
        receivedWaypointPlan.erase(receivedWaypointPlan.begin()); //TODO make pretty
        return tempPlan;
    }

    // Ensure that every other bot has gotten a station plan at this point
    // for (auto &bot : otherBots){
    //     if (bot.get().getStationPlan().empty()){
    //         return {};// If not all bots have a station plan, just go idle
    //     }
    // }
    auto highlevel = HighLevelCBS::get_instance();
    highlevel.findSolution();


    
    return receivedWaypointPlan;
}

std::vector<Point> TestController::findOptimalPath(){
    return sMap.findPath(lastLocation, stationPlan.front());
}

REGISTER_CONTROLLER(TestController, "TestController")