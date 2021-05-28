

#include "UppaalStationGreedyWaypoint.hpp"

/* Definition of the CCI_Controller class. */
#include "argos3/core/control_interface/ci_controller.h"

std::vector<int> UppaalStationGreedyWaypoint::constructStationPlan() {
    return SingleThreadUppaalBot::constructStationPlan();
}

std::vector<int> UppaalStationGreedyWaypoint::constructWaypointPlan() {
    return SingleThreadBotGreedy::constructWaypointPlan();
}

REGISTER_CONTROLLER(UppaalStationGreedyWaypoint, "UppaalStationGreedyWaypoint")