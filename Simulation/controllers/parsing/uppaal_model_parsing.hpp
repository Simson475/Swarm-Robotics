//
// Created by martin on 20/10/20.
//

#ifndef SWARMSIMULATOR_UPPAAL_MODEL_PARSING_H
#define SWARMSIMULATOR_UPPAAL_MODEL_PARSING_H

#include "models/robot/robot.hpp"
#include "models/map/map_structure.hpp"
#include "argos_wrapper/argos_wrapper.hpp"
#include "models/map/line.hpp"
#include "controllers/SingleThreadUppaalBot.hpp"

#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <experimental/iterator>
#include <set>
#include <functional>

struct abs_robot_info {
    std::string name;
    std::vector<int> remaining_stations;

    abs_robot_info(std::string& name, std::vector<int>& remaining_stations) :
    name(name),
    remaining_stations(remaining_stations)
    {}

};

struct robot_at_point : abs_robot_info {
    int current_location;

    robot_at_point(std::string& name, std::vector<int>& remaining_stations, int current_location) :
    abs_robot_info{name, remaining_stations},
    current_location(current_location)
    {}
};

struct robot_moving : abs_robot_info {
    int latest_point;
    int current_target;
    int clock; // Not a float since we do not want that level of detail.

    robot_moving(std::string& name, std::vector<int>& remaining_stations, int latest_point, int current_target,
        int clock) : abs_robot_info(name, remaining_stations), latest_point(latest_point), current_target(current_target),
        clock(clock)
        {}
};


std::string constructUppaalModel(const std::vector<Robot> &robots, Robot &currentRobot, bool stations);


// Functions for getting the information to the Uppaal XML from map_structure
std::vector<abs_robot_info> get_robot_plans_and_positions(const std::vector<Robot> &robots, const Robot &currentRobot);
std::string numOfStations(const Map_Structure &map_structure);
std::string numOfPoints(const Map_Structure &map_structure);
int number_of_robots(const Map_Structure& map_structure);
std::vector<int> get_end_stations(const Map_Structure &map_structure);
std::vector<std::vector<int>> get_distances(const Map_Structure& map_structure);

// Functions for getting the strings needed for the placeholders in the Uppaal XML
void configure_static_settings_of_Uppaal_model(Map_Structure& map_structure);
//std::string current_robots_loc(const std::vector<abs_robot_info>&, const Robot&);

// Helper_functions
std::size_t numOfOtherActiveRobots(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>>&);
std::string get_expanded_distance_matrix(Map_Structure &map_structure, const Point &point);
std::vector<std::vector<float>> getDistanceMatrix(Map_Structure &map_structure);
std::string formatStationOrderLenghts(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots);
std::string formatOrderStartLocs(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots);
std::string formatOtherStationPlan(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots, int numOfStations);
std::string formatOtherOrders(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots, int numOfStations);


//********************************* Formatting functions:
// Help-function for delimiter
template <typename C>
std::string element_joiner(std::vector<C> elements, std::string delimiter, std::string prefix, std::string postfix){
    std::stringstream formatted_elements{};

    formatted_elements << prefix;
    std::copy(elements.begin(),
              elements.end(),
              std::experimental::make_ostream_joiner(formatted_elements, delimiter));
    formatted_elements << postfix;

    return formatted_elements.str();
}

// Function to convert a vector of IDs to a int (0's and 1's) vector where indexes indicate that the ID was present
template <typename C>
std::vector<int> convertIDsToBools(int size, C ids){
    std::vector<int> verbatimOrder(size, 0);

    for(int id = 0; id < size; id++){
        if(std::find(ids.begin(), ids.end(), id) != ids.end())
            verbatimOrder[id] = 1;
    }

    return verbatimOrder;
}

// Takes a matrix of values (std::to_string must work on the type) and convert the lines to a string.
template <typename C>
std::vector<std::vector<std::string>> formatMatrixLines(const std::vector<std::vector<C>> &dist_matrix){
    std::vector<std::vector<std::string>> matrixStr{};

    for(auto& line : dist_matrix){
        std::vector<std::string> formatted_numbers{};
        for(auto& number : line) {
            formatted_numbers.push_back(std::to_string(number));
        }
        matrixStr.push_back(formatted_numbers);
    }

    return matrixStr;
}


// Gets distance Matrix between all stations.
std::string getStationDistanceMatrix(Map_Structure &map_structure);
// Formats and combine the rows of a matrix into a single formatted string.
std::string combineMatrixLines(const std::vector<std::vector<std::string>> &distance_values);
// Function for formatting the order array
std::string format_order(int numOfStations, std::set<int> order);
// Endstations formatting
std::string format_endstations(int numOfStations, std::vector<int> endstationIDs);
// Get the right number of entries in the query
std::string format_query(unsigned numOfPoint);

// Function for formatting an arbitrary matrix where the type has a std::to_string(C) function.
template <typename C>
std::string formatMatrix(const std::vector<std::vector<C>> &distance_matrix){
    return combineMatrixLines(formatMatrixLines(distance_matrix));
}

#endif //SWARMSIMULATOR_UPPAAL_MODEL_PARSING_H
