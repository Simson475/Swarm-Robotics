

#include "GreedyStationUppaalWaypoint.hpp"

/* Definition of the CCI_Controller class. */
#include "argos3/core/control_interface/ci_controller.h"

std::vector<int> GreedyStationUppaalWaypoint::constructStationPlan() {
    return SingleThreadBotGreedy::constructStationPlan();
}

std::vector<int> GreedyStationUppaalWaypoint::constructWaypointPlan() {
    return SingleThreadUppaalBot::constructWaypointPlan();
}

REGISTER_CONTROLLER(GreedyStationUppaalWaypoint, "GreedyStationUppaalWaypoint")