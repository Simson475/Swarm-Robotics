#include "GreedyStationCBSWaypoint.hpp"

std::vector<int> GreedyStationCBSWaypoint::constructStationPlan() {
    return SingleThreadBotGreedy::constructStationPlan();
}

std::vector<int> GreedyStationCBSWaypoint::constructWaypointPlan() {
    return SingleThreadBotCBS::constructWaypointPlan();
}

//REGISTER_CONTROLLER(GreedyStationCBSWaypoint, "GreedyStationCBSWaypoint")