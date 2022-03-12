#include "SingleThreadBotCBS.hpp"

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

void SingleThreadBotCBS::specialInit(){
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

//REGISTER_CONTROLLER(SingleThreadBotCBS, "SingleThreadBotCBS")