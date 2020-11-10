#include "CTrajectoryLoopFunctions.h"

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
    std::cout << "Setup complete" << std::endl;
}

/****************************************/
/****************************************/
REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "CTrajectoryLoopFunctions")
