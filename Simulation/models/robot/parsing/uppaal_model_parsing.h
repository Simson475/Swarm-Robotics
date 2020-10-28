//
// Created by martin on 20/10/20.
//

#ifndef SWARMSIMULATOR_UPPAAL_MODEL_PARSING_H
#define SWARMSIMULATOR_UPPAAL_MODEL_PARSING_H

#include "models/robot/robot.hpp"
#include <string>
#include <vector>
#include <map>
#include <cmath>

struct abs_robot_info {
    std::string name;
    std::vector<int> remaining_stations;

    abs_robot_info(std::string name, std::vector<int> remaining_stations) :
    name(name),
    remaining_stations(remaining_stations)
    {}

};

struct robot_at_point : abs_robot_info {
    int current_location;

    robot_at_point(std::string name, std::vector<int> remaining_stations, int current_location) :
    abs_robot_info{name, remaining_stations},
    current_location(current_location)
    {}
};

struct robot_moving : abs_robot_info {
    int latest_point;
    int current_target;
    int clock; // Not a float since we do not want that level of detail.

    robot_moving(std::string name, std::vector<int> remaining_stations, int latest_point, int current_target,
        int clock) : abs_robot_info(name, remaining_stations), latest_point(latest_point), current_target(current_target),
        clock(clock)
        {}
};

std::string constructUppaalModel(std::vector<Robot> &robots, Robot &currentRobot, bool stations);

#endif //SWARMSIMULATOR_UPPAAL_MODEL_PARSING_H
