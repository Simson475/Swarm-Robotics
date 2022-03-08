

#include "GreedyStationCBSWaypoint.hpp"

/* Definition of the CCI_Controller class. */
#include "argos3/core/control_interface/ci_controller.h"

std::vector<int> GreedyStationCBSWaypoint::constructStationPlan() {
    return SingleThreadBotGreedy::constructStationPlan();
}

std::vector<int> GreedyStationCBSWaypoint::constructWaypointPlan() {
    return SingleThreadBotCBS::constructWaypointPlan();
}

REGISTER_CONTROLLER(GreedyStationCBSWaypoint, "GreedyStationCBSWaypoint")