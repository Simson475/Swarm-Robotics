#include "CTrajectoryLoopFunctions.h"

/****************************************/
/****************************************/

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const argos::Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const argos::Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::Init(argos::TConfigurationNode &t_tree) {
    /*
     * Go through all the robots in the environment
     * and create an entry in the waypoint map for each of them
     */
    /* Get the map of all foot-bots from the space */

    argos::CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for (auto it = tFBMap.begin(); it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current foot-bot */
        argos::CFootBotEntity *pcFB = argos::any_cast<argos::CFootBotEntity*>(it->second);
        /* Create a waypoint vector */
        m_tWaypoints[pcFB] = std::vector<argos::CVector3>();
        /* Add the initial position of the foot-bot */
        m_tWaypoints[pcFB].push_back(
            pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
    }

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

void CTrajectoryLoopFunctions::Reset() {
    /*
     * Clear all the waypoint vectors
     */
    /* Get the map of all foot-bots from the space */
    argos::CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for (auto it = tFBMap.begin(); it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current foot-bot */
        argos::CFootBotEntity *pcFB = argos::any_cast<argos::CFootBotEntity*>(it->second);
        /* Clear the waypoint vector */
        m_tWaypoints[pcFB].clear();
        /* Add the initial position of the foot-bot */
        m_tWaypoints[pcFB].push_back(
            pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
    }
}

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::PostStep() {
    /* Get the map of all foot-bots from the space */
    argos::CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for (auto it = tFBMap.begin(); it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current foot-bot */
        argos::CFootBotEntity *pcFB = argos::any_cast<argos::CFootBotEntity*>(it->second);
        /* Add the current position of the foot-bot if it's sufficiently far from
         * the last */
        if (SquareDistance(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position,
                           m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED) {
            m_tWaypoints[pcFB].push_back(
                pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
        }
    }

}
/****************************************/
/****************************************/
REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "CTrajectoryLoopFunctions")
