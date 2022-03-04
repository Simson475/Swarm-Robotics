

#include "CBSController.hpp"

/* Definition of the CCI_Controller class. */
#include "argos3/core/control_interface/ci_controller.h"

std::vector<int> CBSController::constructStationPlan() {
    return SingleThreadBotGreedy::constructStationPlan();
}

std::vector<int> CBSController::constructWaypointPlan() {
    return SingleThreadBotCBS::constructWaypointPlan();
}

REGISTER_CONTROLLER(CBSController, "CBSController")