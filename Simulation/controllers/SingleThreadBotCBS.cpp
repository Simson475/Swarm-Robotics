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
  if ( !receivedWaypointPlan.empty()){
    return receivedWaypointPlan;
  }
  auto highlevel = HighLevelCBS::get_instance();
  highlevel.findSolution();
  
  return receivedWaypointPlan;
}

void specialInit(){
  if (jobGenerator->anyJobsLeft()) {
    log_helper("Sets job");
    setJob();
    experiment_job_data("StartedJob", currentJob->getID(),
                        argos::CSimulator::GetInstance().GetSpace().GetSimulationClock());
  }
  else{
    throw "No jobs at the beginning of simulation!";
  }
}


REGISTER_CONTROLLER(SingleThreadBotCBS, "SingleThreadBotCBS_controller")