//
// Created by martin on 20/10/20.
//

#include "uppaal_model_parsing.hpp"
#include "argos_wrapper/argos_wrapper.hpp"

#include <cmath>

/**
 * Converts a matrix from vector<vector<string>> to string (in the {\n{a,b,c}\n,\n{d,e,f}\n} format)
 * 
 * @param distance_values std::vector<std::vector<std::string>> &
 * 
 * @return string
*/
std::string combineMatrixLines(const std::vector<std::vector<std::string>> &distance_values){
    // Temporary results of the lines in the matrix on string-form.
    std::vector<std::string> waited_matrix{};

    // Gets all the line with correct post-/prefix and delimitor.
    for(auto& dist_line : distance_values){
        std::string line = element_joiner(dist_line, ", ", "{", "}");
        waited_matrix.emplace_back(line);// Appends to the vector
    }

    std::string final_matrix = element_joiner(waited_matrix, ",\n", "{\n", "\n}");

    return final_matrix;
}

/**
 * Returns the number of other robots
 * 
 * @param otherBots std::vector<std::reference_wrapper<RobotInterface>> &
 * 
 * @return size_t
*/
std::size_t numOfOtherRobots(const std::vector<std::reference_wrapper<RobotInterface>>& otherBots){
    return otherBots.size();
}

/**
 * Returns the number of active other robots
 *
 * @param otherBots std::vector<std::reference_wrapper<RobotInterface>> &
 *
 * @return size_t
*/
std::size_t numOfOtherActiveRobots(const std::vector<std::reference_wrapper<RobotInterface>>& otherBots){
    std::size_t active_robots = 0;

    for(auto& bot : otherBots){
        if (bot.get().isActive()){
            active_robots++;
        }
    }

    return active_robots;
}

/**
 * Gets the distances between all stations and the point given as argument
 * 
 * @param map_structure Map_Structure &
 * @param p_id int
 * 
 * @return string - matrix in string format
*/ 
std::string get_expanded_distance_matrix(Map_Structure &map_structure, int p_id){
    // Copies the full distance matrix and the short distance between stations.
    const std::vector<std::vector<float>>& fullDistMatrix = map_structure.getShortestDistanceMatrix();
    std::vector<std::vector<float>> newDistMatrix = map_structure.floydShortestOfStations();

    // Adds the distance from stations to point by extending distance matrix by one column
    for(std::size_t i = 0; i < newDistMatrix.size(); i++){
        newDistMatrix[i].push_back(fullDistMatrix[i][p_id]);// push_back adds the element to the end
    }

    // Adding distances from point to stations in the matrix by extending distance matrix by one row
    std::vector<float> pointToStations(newDistMatrix.size() + 1, 0);
    for(std::size_t i = 0; i < newDistMatrix.size(); i++){
        pointToStations[i] = fullDistMatrix[p_id][i];
    }
    newDistMatrix.push_back(pointToStations);// push_back adds the element to the end

    return formatMatrix(newDistMatrix);
}

/**
 * Formats the order as a vector string {a,b,c} where each index is 1 if the id is present and 0 if not.
 * 
 * @param numOfStations int
 * @param order std::set<int>
 * 
 * @return std::string
*/
std::string format_order(int numOfStations, std::set<int> order){
    std::vector<int> verbatimOrder = convertIDsToBools<std::set<int>>(numOfStations, std::move(order));

    std::string formatted_order = element_joiner(verbatimOrder, ", ", "{", "}");

    return formatted_order;
}

/**
 * Formats the ids as a vector string {a,b,c} where each index is 1 if the id is present and 0 if not.
 * 
 * @param numOfStations int
 * @param endstationIDs std::set<int>
 * 
 * @return std::string
*/
std::string format_endstations(int numOfStations, std::set<int> endstationIDs){
    std::vector<int> verbatimOrder = convertIDsToBools(numOfStations, std::move(endstationIDs));

    std::string formatted_endstations = element_joiner(verbatimOrder, ", ", "{", "}");

    return formatted_endstations;
}

/**
 * @TODO What does this do?
 * 
 * @param map_structure Map_Structure &
 * 
 * @return std::vector<std::vector<float>>
*/
std::vector<std::vector<float>> getDistanceMatrix(Map_Structure &map_structure){
    // We want to construct a square matrix?
    int sizeLines = std::sqrt(map_structure.lines.size()); //@todo: Make 2-dimentional to begin with.
    std::vector<std::vector<float>> waypointsDistances(sizeLines, std::vector<float>());

    int k = -1;
    for (long unsigned i = 0; i < map_structure.lines.size(); i++) {
        if (i % sizeLines == 0)
            k++;
        float distance = map_structure.lines[i].GetDistance();
        distance = distance == -1 ? 0 : distance / (double)VELOCITY*100;
        waypointsDistances[k].push_back(distance);
    }

    return waypointsDistances;
}

/**
 * Returns vector string of the active other bots station plan size
 * 
 * @param otherBots std::vector<std::reference_wrapper<RobotInterface>> &
 * 
 * @return std::string
*/
std::string formatStationOrderLenghts(const std::vector<std::reference_wrapper<RobotInterface>> &otherBots){
    std::vector<unsigned> orderLenghts{};

    for(auto& bot: otherBots){
        if(bot.get().isActive())
            orderLenghts.push_back(bot.get().sizeOfStationPlan());
    }

    return element_joiner(orderLenghts, ", ", "{", "}");
}

std::string formatWaypointOrderLenghts(const std::vector<std::reference_wrapper<RobotInterface>> &otherBots){
    std::vector<unsigned> orderLenghts{};

    for(auto& bot: otherBots){
        if(bot.get().isActive())
            orderLenghts.push_back(bot.get().getWaypointPlan().size());
    }

    return element_joiner(orderLenghts, ", ", "{", "}");
}

std::string formatOrthersStartLocs(const std::vector<std::reference_wrapper<RobotInterface>> &otherBots){
    std::vector<int> otherLocations{};

    for(auto& bot: otherBots) {
        if (bot.get().isActive()){
            if (bot.get().isWorking())
                otherLocations.push_back(bot.get().getLastLocation());
            else
                otherLocations.push_back(0);
        }
    }

    return element_joiner(otherLocations, ", ", "{", "}");
}

std::string formatOrtherWaypointStartLocs(const std::vector<std::reference_wrapper<RobotInterface>> &otherBots){
    std::vector<int> otherLocations{};

    for(auto& bot: otherBots) {
        if (bot.get().isActive()){
            otherLocations.push_back(bot.get().getLastLocation());
        }
    }

    return element_joiner(otherLocations, ", ", "{", "}");
}

std::string formatOtherStationPlan(const std::vector<std::reference_wrapper<RobotInterface>> &otherBots, int numOfStations){
    std::vector<std::vector<int>> plans{};

    for(auto& bot: otherBots){
        if(bot.get().isActive()) {
            std::vector<int> stationPlan{};
            for (int station : bot.get().getStationPlan()) {
                stationPlan.push_back(station);
            }
            stationPlan.resize(numOfStations);
            plans.push_back(stationPlan);
        }
    }

    return formatMatrix(plans);
}

std::string formatOtherWaypointPlan(const std::vector<std::reference_wrapper<RobotInterface>> &otherBots, int numOfStations){
    std::vector<std::vector<int>> plans{};

    for(auto& bot: otherBots){
        if(bot.get().isActive()) {
            std::vector<int> waypointPlan{};
            for (int pointID : bot.get().getWaypointPlan()) {
                waypointPlan.push_back(pointID);
            }
            waypointPlan.resize(numOfStations);
            plans.push_back(waypointPlan);
        }
    }

    return formatMatrix(plans);
}

std::string formatOtherOrders(const std::vector<std::reference_wrapper<RobotInterface>> &otherBots, int numOfStations){
    std::vector<std::vector<int>> orders{};

    for(auto& bot: otherBots){
        if(bot.get().isActive()) {
            std::vector<int> order = convertIDsToBools(numOfStations, bot.get().getOrder());
            orders.push_back(order);
        }
    }

    return formatMatrix(orders);
}

std::string formatOtherStationDistances(const std::vector<std::reference_wrapper<RobotInterface>> &otherBots, Map_Structure map_structure){
    std::vector<std::string> distances{};

    for(auto& bot: otherBots){
        if(bot.get().isActive()) {
            double dist;
            if(bot.get().isWorking()){
                dist = 0.0;
            }
            else {
                dist = getDistanceToNextPoint(bot.get(), map_structure, bot.get().getNextWaypoint());
                dist += getDistanceBetweenPoints(map_structure, bot.get().getWaypointPlan());
            }
            std::string tmp = std::to_string(dist);
            std::replace(tmp.begin(), tmp.end(), ',', '.');
            distances.push_back(tmp);
        }
    }

    return element_joiner(distances, ", ", "{", "}");
}

std::string formatOtherWaypointDistances(const std::vector<std::reference_wrapper<RobotInterface>> &otherBots, Map_Structure map_structure){
    std::vector<std::string> distances{};

    for(auto& bot: otherBots){
        if(bot.get().isActive()) {
            double dist;
            if(bot.get().isWorking()){
                dist = 0.0;
            }
            else {
                dist = getDistanceToNextPoint(bot.get(), map_structure, bot.get().getNextWaypoint());
            }
            std::string tmp = std::to_string(dist);
            std::replace(tmp.begin(), tmp.end(), ',', '.');
            distances.push_back(tmp);
        }
    }

    return element_joiner(distances, ", ", "{", "}");
}

std::string formatOtherWaypointOrders(const std::vector<std::reference_wrapper<RobotInterface>> &otherBots, int numOfStations){
    std::vector<std::vector<int>> orders{};

    for(auto& bot: otherBots){
        if(bot.get().isActive()) {
            std::vector<int> order = convertIDsToBools(numOfStations, std::vector<int>(1, bot.get().getNextStation()));
            orders.push_back(order);
        }
    }

    return formatMatrix(orders);
}

std::string formatOtherWorking(const std::vector<std::reference_wrapper<RobotInterface>> &otherBots){
    std::vector<bool> robotWorking{};

    for (auto& robot : otherBots) {
        if (robot.get().isActive()) {
            robot.get().isWorking() ? robotWorking.push_back(true) : robotWorking.push_back(false);
        }
    }

    return element_joiner(robotWorking, ", ", "{", "}");
}

std::string formatWorkedTime(const std::vector<std::reference_wrapper<RobotInterface>> &otherBots){
    std::vector<int> robotWorkedTime{};

    for (auto& robot : otherBots) {
        if (robot.get().isActive()) {
            robot.get().isWorking() ? robotWorkedTime.push_back(robot.get().getClockCount()) : robotWorkedTime.push_back(0);
        }
    }

    return element_joiner(robotWorkedTime, ", ", "{", "}");
}

std::string format_state_vars(unsigned numOfPoint){
    std::vector<int> points(numOfPoint);
    std::iota(std::begin(points), std::end(points), 0);

    return element_joiner(points, "],\nvisited[", "visited[", "]\n");
}

std::string format_point_vars(unsigned numOfPoint){
    std::vector<int> points(numOfPoint);
    std::iota(std::begin(points), std::end(points), 0);

    return element_joiner(points, "],\npoint_touched[", "point_touched[", "]\n");
}
