#include "TestController.hpp"
#include <iostream>

std::vector<int> TestController::constructStationPlan(){
    std::vector<int> vector{};
    switch (temp%3){
        case 0: vector={6};
            break;
        case 1: vector={9};
            break;
        case 2: vector= {33};
            break;
        default: vector = SingleThreadBotGreedy::constructStationPlan();
    }
    temp++;
    std::cout << sMap.getPointByID(30) << "<<-- 30 72 --> " << sMap.getPointByID(72);
        auto temp =sMap.findPath(36,71);
    for(size_t i=0; i < temp.size();i++){
            std::cout << "\n\n\n\n\n";
            std::cout << this->GetId()<<"\n";
            for(size_t i=0; i < temp.size();i++){
               std::cout << temp[i].getId() << " ";
            }
            std::cout << "\n\n\n\n\n";
    }
    return SingleThreadBotGreedy::constructStationPlan();
}
std::vector<int> TestController::constructWaypointPlan(){
    // for(size_t i=0; i < receivedWaypointPlan.size();i++){
    //         std::cout << "\n\n\n\n\n";
    //         std::cout << this->GetId()<<"\n";
    //         for(size_t i=0; i < receivedWaypointPlan.size();i++){
    //            std::cout << receivedWaypointPlan[i] << " ";
    //         }
    //         std::cout << "\n\n\n\n\n";
    // }
    if ( !receivedWaypointPlan.empty()){
        std::vector<int> tempPlan = receivedWaypointPlan;
        receivedWaypointPlan.erase(receivedWaypointPlan.begin()); //TODO make pretty
        return tempPlan;
    }
    auto highlevel = HighLevelCBS::get_instance();
    highlevel.findSolution();

    return receivedWaypointPlan;
}

std::vector<Point> TestController::findOptimalPath(){

    return sMap.findPath(lastLocation, stationPlan.front());
}

void TestController::specialInit(){
    subtype = "CBS";
}

REGISTER_CONTROLLER(TestController, "TestController")