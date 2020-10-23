//
// Created by martin on 20/10/20.
//

#ifndef SWARMSIMULATOR_UPPAAL_MODEL_PARSING_H
#define SWARMSIMULATOR_UPPAAL_MODEL_PARSING_H

#include "models/robot/robot.hpp"
#include <string>
#include <vector>
#include <map>

std::string constructUppaalModel(std::vector<Robot> &robots, Robot &currentRobot, bool stations);

#endif //SWARMSIMULATOR_UPPAAL_MODEL_PARSING_H
