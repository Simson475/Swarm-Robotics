#include "CTrajectoryLoopFunctions.h"

#include "controllers/SingleThreadUppaalBot.hpp"

#include <set>

/****************************************/
/****************************************/


/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::Init(argos::TConfigurationNode &t_tree) {
    Map_Structure &sMap = Map_Structure::get_instance();
    std::cout << "Set Path" << std::endl;
    sMap.setFolderPath();
    std::cout << "Make Path" << std::endl;
    if(newJobs)
        sMap.generateJobs(); // call this for generating new list of jobs
    std::cout << "Set Stations" << std::endl;
    sMap.initializeStations();
    std::cout << "Set Jobs" << std::endl;
    sMap.initializeJobs();
    std::cout << "Set Waypoints" << std::endl;
    sMap.collectAllWayPoints();
    std::cout << "Set Robot Folder" << std::endl;
    sMap.createFolderForEachRobot();
    std::cout << "Set Lines" << std::endl;
    sMap.setAllPossibleLines();
    std::cout << "Calculating Distance Matrix" << std::endl;
    sMap.setDistanceMatrix();
    std::cout << "Create JSON" << std::endl;
    sMap.createStaticJSON();

    std::cout << "Setting JobGenerator in controllers" << std::endl;
    setJobGenerator();
    assignJobGeneratorToControllers();

    std::cout << "Setup complete" << std::endl;
}

void CTrajectoryLoopFunctions::setJobGenerator(){
    Map_Structure &sMap = Map_Structure::get_instance();

    int numOfStations = sMap.endStationIDs.size() + sMap.endStationIDs.size();

    std::set<int> endStationIDs{};
    for(auto id : sMap.endStationIDs)
        endStationIDs.insert(id);

    jobGenerator = std::make_shared<JobGenerator>(JobGenerator(numOfStations, endStationIDs, 2));
};

void CTrajectoryLoopFunctions::assignJobGeneratorToControllers() {
    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto it = tBotMap.begin(); it != tBotMap.end();
         ++it) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(it->second);
        SingleThreadUppaalBot& controller = dynamic_cast<SingleThreadUppaalBot&>(pcBot->GetControllableEntity().GetController());

        controller.setJobGenerator(jobGenerator);
    }
}
/****************************************/
/****************************************/
REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "CTrajectoryLoopFunctions")
