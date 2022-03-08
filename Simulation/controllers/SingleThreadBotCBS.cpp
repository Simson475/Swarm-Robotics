#include "SingleThreadBotCBS.hpp"
#include "argos_wrapper/argos_wrapper.hpp"
#include "HighLevelCBS.hpp"

#include <exception>
#include <cstdio>
#include <regex>
#include <fstream>
#include <set>
#include <iostream>
#include <filesystem>
#include <ctime>
#include <chrono>
#include <iterator>

void SingleThreadBotCBS::Init(argos::TConfigurationNode &t_node){
    RobotInterface::Init(t_node);
}



std::vector<int> SingleThreadBotCBS::constructStationPlan() {
    throw new std::runtime_error("SingleThreadBotCBS::constructStationPlan() not implemented");
}


std::vector<int> SingleThreadBotCBS::constructWaypointPlan() {
    auto highlevel = HighLevelCBS::get_instance();
    // if (!highlevel.readyforcalculating){
    //     highlevel.findSolution();
    //     highlevel.calculating = true;
    // }
    return receivedWaypointPlan;
}


REGISTER_CONTROLLER(SingleThreadBotCBS, "SingleThreadBotCBS_controller")