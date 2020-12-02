//
// Created by martin on 29/10/20.
//

#ifndef SWARMSIMULATOR_ARGOS_WRAPPER_H
#define SWARMSIMULATOR_ARGOS_WRAPPER_H

#include "models/map/map_structure.hpp"
/* Definition of the CCI_Controller class. */
#include "argos3/core/control_interface/ci_controller.h"

double getDistanceToNextPoint(const argos::CCI_Controller& controller, Map_Structure& map_structure, int nextPoint);
double getDistanceBetweenPoints(Map_Structure& map_structure, std::vector<int> points);

#endif //SWARMSIMULATOR_ARGOS_WRAPPER_H
