#include "CTrajectoryLoopFunctions.h"

#include "controllers/SingleThreadUppaalBot.hpp"

#include <set>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <filesystem>

/****************************************/
/****************************************/


/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::Init(argos::TConfigurationNode &t_tree) {
    Map_Structure &sMap = Map_Structure::get_instance();
    std::cout << "Set Path" << std::endl;
    sMap.setFolderPath();
    std::cout << "Set Stations" << std::endl;
    sMap.initializeStations();
    std::cout << "Set Waypoints" << std::endl;
    sMap.collectAllWayPoints();
    std::cout << "Set Robot Folder" << std::endl;
    sMap.createFolderForEachRobot();
    std::cout << "Set Lines" << std::endl;
    sMap.setAllPossibleLines();
    std::cout << "Calculating Distance Matrix" << std::endl;
    sMap.setDistanceMatrix();

    std::cout << "Setting JobGenerator in controllers" << std::endl;
    initJobGenerator();
    assignJobGeneratorToControllers();

    std::cout << "Setting bots initial location" << std::endl;
    setInitLocationOnControllers(sMap);

    std::cout << "Controllers get references to other controllers" << std::endl;
    haveControllersAccessEachOther();

    std::cout << "Deletes old log file" << std::endl;
    removeOldLogFile();
    std::cout << "Setup complete" << std::endl;
}

bool CTrajectoryLoopFunctions::IsExperimentFinished() {
    return jobGenerator->allJobsCompleted();
}

void CTrajectoryLoopFunctions::PostExperiment() {
    std::ofstream logFile;
    logFile.open(std::string{std::filesystem::current_path()} + "/log.txt", std::ofstream::app);

    logFile << "Simulation completed at time step: ";
    logFile << argos::CSimulator::GetInstance().GetSpace().GetSimulationClock() << std::endl;

    logFile.close();
}

void CTrajectoryLoopFunctions::initJobGenerator(){
    Map_Structure &sMap = Map_Structure::get_instance();

    int numOfStations = sMap.stationIDs.size() + sMap.endStationIDs.size();

    std::set<int> endStationIDs{};
    for(auto id : sMap.endStationIDs)
        endStationIDs.insert(id);

    jobGenerator = std::make_shared<JobGenerator>(JobGenerator(numOfStations, endStationIDs, 3));
};

void CTrajectoryLoopFunctions::assignJobGeneratorToControllers() {
    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        auto& controller = dynamic_cast<SingleThreadUppaalBot&>(pcBot->GetControllableEntity().GetController());

        controller.setJobGenerator(jobGenerator);
    }
}

void CTrajectoryLoopFunctions::setInitLocationOnControllers(Map_Structure& sMap){
    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        auto& controller = dynamic_cast<SingleThreadUppaalBot&>(pcBot->GetControllableEntity().GetController());

        Robot robot = sMap.getRobotByName(controller.GetId());

        controller.setInitLocation(robot.getInitialLoc().getId());
    }
}

void CTrajectoryLoopFunctions::haveControllersAccessEachOther(){
    Map_Structure &sMap = Map_Structure::get_instance();

    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        auto &controller = dynamic_cast<SingleThreadUppaalBot &>(pcBot->GetControllableEntity().GetController());

        controller.obtainOtherBots(sMap);
    }
}

void CTrajectoryLoopFunctions::removeOldLogFile() {
    std::string fileName = std::string{std::filesystem::current_path()} + "/log.txt";
    remove(fileName.c_str());
}
/****************************************/
/****************************************/
REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "CTrajectoryLoopFunctions")
