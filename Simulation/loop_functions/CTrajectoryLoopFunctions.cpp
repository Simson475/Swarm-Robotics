#include "CTrajectoryLoopFunctions.h"

#include "controllers/RobotInterface/RobotInterface.hpp"

#include <set>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <filesystem>
#include <exception>

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
    removeFoldersAndPlan();
    setRobotFolders();
    std::cout << "Remove bad points" << std::endl;
    sMap.eliminateBadPoints();
    std::cout << "Set Lines" << std::endl;
    sMap.setAllPossibleLines();
    std::cout << "Calculating Distance Matrix" << std::endl;
    sMap.setDistanceMatrix();
    sMap.setRealDistanceMatrix();

    std::cout << "Setting JobGenerator in controllers" << std::endl;
    initJobGenerator();
    assignJobGeneratorToControllers();

    std::cout << "Setting bots initial location" << std::endl;
    setInitLocationOnControllers(sMap);

    std::cout << "Controllers get references to other controllers" << std::endl;
    haveControllersAccessEachOther();

    std::cout << "Deletes old log file" << std::endl;
    removeOldLogFile();

    std::cout << "Prepare csv-file for data collection" << std::endl;
    setUpDataCollection();
    std::cout << "Setup complete" << std::endl;
}

bool CTrajectoryLoopFunctions::IsExperimentFinished() {
    return jobGenerator->allJobsCompleted();
}

void CTrajectoryLoopFunctions::PreStep() {
    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        auto& controller = dynamic_cast<RobotInterface&>(pcBot->GetControllableEntity().GetController());

        if(controller.isInLivelock()) {
            std::ofstream logFile;
            logFile.open(std::string{std::filesystem::current_path()} + "/log.txt", std::ofstream::app);

            logFile << "Robot in livelock: " + controller.GetId() << std::endl;
            logFile.close();

            fprintf(stderr,"Robot %s is in a livelock", controller.GetId().c_str());
            exit(EXIT_FAILURE);
        }
    }
}

void CTrajectoryLoopFunctions::PostExperiment() {
    std::ofstream logFile;
    logFile.open(std::string{std::filesystem::current_path()} + "/log.txt", std::ofstream::app);

    logFile << "Simulation completed at time step: ";
    logFile << argos::CSimulator::GetInstance().GetSpace().GetSimulationClock() << std::endl;

    logFile.close();


    std::ofstream dataFile;
    dataFile.open(std::string{std::filesystem::current_path()} + "/data.csv", std::ofstream::app);

    dataFile << ", " << "LogicalTotal, " << argos::CSimulator::GetInstance().GetSpace().GetSimulationClock() << ", , ," << std::endl;
    dataFile.close();
}

void CTrajectoryLoopFunctions::initJobGenerator(){
    Map_Structure &sMap = Map_Structure::get_instance();

    int robotCount = argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot").size();

    // Use spawn locations as delivery stations
    int spawnPointOffset = sMap.endStationIDs.size() + sMap.stationIDs.size();
    std::set<int> endStationIDs{};
    for(int i = 0; i < robotCount; ++i)
        endStationIDs.insert(spawnPointOffset + i);

    int jobs_per_robot = 2000;
    try {
        argos::TConfigurationNode &t_node = argos::CSimulator::GetInstance().GetConfigurationRoot();
        argos::TConfigurationNode &params = argos::GetNode(t_node, "custom");
        argos::GetNodeAttribute(params, "jobs_per_robot", jobs_per_robot);

        jobs_per_robot *= robotCount;
        std::cout << "Number of jobs are: " << jobs_per_robot << std::endl;
    }
    catch (argos::CARGoSException &e){
        std::cout << "Number of jobs defaulted to: " << jobs_per_robot << std::endl;
    }

    jobGenerator = std::make_shared<UniqueStationsJobGenerator>(UniqueStationsJobGenerator(sMap.getAmountOfStations(), endStationIDs, jobs_per_robot, sMap.endStationIDs.size()));
    // jobGenerator = std::make_shared<PredefinedJobGenerator>(PredefinedJobGenerator(sMap.getAmountOfStations(), endStationIDs, jobs_per_robot));
};


void CTrajectoryLoopFunctions::removeFoldersAndPlan(){
    std::string path = std::filesystem::current_path();
    for (const auto & entry : std::filesystem::directory_iterator(path)) {
        if (entry.path().filename().string().find("fb") != std::string::npos
         || entry.path().filename().string().find("jobProgress") != std::string::npos)
            std::filesystem::remove_all(entry.path());
    }
}

void CTrajectoryLoopFunctions::setRobotFolders(){
    std::string currentFolder = std::filesystem::current_path();

    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        auto &controller = dynamic_cast<RobotInterface &>(pcBot->GetControllableEntity().GetController());

        std::string temp = currentFolder + "/" + controller.GetId();

        if (mkdir(temp.c_str(), 0777) == -1) {
            throw std::runtime_error("Cannot write folder");
        }
    }
}

void CTrajectoryLoopFunctions::assignJobGeneratorToControllers() {
    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        auto& controller = dynamic_cast<RobotInterface&>(pcBot->GetControllableEntity().GetController());

        controller.setJobGenerator(jobGenerator);
    }
}

void CTrajectoryLoopFunctions::setInitLocationOnControllers(Map_Structure& sMap){
    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        auto &controller = dynamic_cast<RobotInterface &>(pcBot->GetControllableEntity().GetController());

        //Robot robot = sMap.getRobotByName(controller.GetId());

        //TODO Needs improvement!
        for (auto &p : sMap.points) {
            if(p.getName() == "S." + controller.GetId()) {
                controller.setInitLocation(p.getId());
                break;
            }

        }
    }
}

void CTrajectoryLoopFunctions::haveControllersAccessEachOther(){
    Map_Structure &sMap = Map_Structure::get_instance();

    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        auto &controller = dynamic_cast<RobotInterface &>(pcBot->GetControllableEntity().GetController());

        controller.obtainOtherBots(sMap);
    }
}

void CTrajectoryLoopFunctions::removeOldLogFile() {
    std::string fileName = std::string{std::filesystem::current_path()} + "/log.txt";
    remove(fileName.c_str());
}

void CTrajectoryLoopFunctions::setUpDataCollection(){
    std::ofstream logFile;
    logFile.open(std::string{std::filesystem::current_path()} + "/data.csv");

    logFile << "Robot, Type, Time, currentPos, Value_1, Value_2" << std::endl;
    logFile.close();
}

/****************************************/
/****************************************/
REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "CTrajectoryLoopFunctions")
