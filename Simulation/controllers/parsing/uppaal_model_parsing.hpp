//
// Created by martin on 20/10/20.
//

#ifndef SWARMSIMULATOR_UPPAAL_MODEL_PARSING_H
#define SWARMSIMULATOR_UPPAAL_MODEL_PARSING_H

#include "models/map/map_structure.hpp"
#include "controllers/SingleThreadUppaalBot.hpp"

#include <string>
#include <vector>
#include <sstream>
#include <experimental/iterator>
#include <set>

// Helper_functions
std::size_t numOfOtherActiveRobots(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>>&);
std::string get_expanded_distance_matrix(Map_Structure &map_structure, int p_id);
std::vector<std::vector<float>> getDistanceMatrix(Map_Structure &map_structure);
std::string formatStationOrderLenghts(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots);
std::string formatOrthersStartLocs(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots);
std::string formatOtherStationPlan(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots, int numOfStations);
std::string formatOtherOrders(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots, int numOfStations);
std::string formatOtherStationDistances(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots, Map_Structure map_structure);
std::string formatOtherWorking(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots);
std::string formatWorkedTime(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots);

// Waypoint planning helper functions
std::string formatWaypointOrderLenghts(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots);
std::string formatOrtherWaypointStartLocs(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots);
std::string formatOtherWaypointPlan(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots, int numOfStations);
std::string formatOtherWaypointOrders(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots, int numOfStations);
std::string formatOtherWaypointDistances(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots, Map_Structure map_structure);

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

// Formats and combine the rows of a matrix into a single formatted string.
std::string combineMatrixLines(const std::vector<std::vector<std::string>> &distance_values);
// Function for formatting the order array
std::string format_order(int numOfStations, std::set<int> order);
// Endstations formatting
std::string format_endstations(int numOfStations, std::set<int> endstationIDs);
// Get the right number of entries in the query
std::string format_query(unsigned numOfPoint);

// Function for formatting an arbitrary matrix where the type has a std::to_string(C) function.
template <typename C>
std::string formatMatrix(const std::vector<std::vector<C>> &distance_matrix){
    return combineMatrixLines(formatMatrixLines(distance_matrix));
}

#endif //SWARMSIMULATOR_UPPAAL_MODEL_PARSING_H
