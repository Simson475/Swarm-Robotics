//
// Created by martin on 20/10/20.
//

#include "uppaal_model_parsing.hpp"

#include <cmath>


std::vector<abs_robot_info> get_robot_plans_and_positions(const std::vector<Robot> &robots, const Robot &currentRobot){
    // What we need from the other robots are:
    // - Their plan
    // - Current location: either point or edge including time spend on the edge.
    std::map<std::string, std::vector<int>> plans{};
    std::vector<abs_robot_info> current_plans_and_locations{};

    // This gets the plans of the robots.
    for(auto& robot: robots){
        plans.insert({robot.getName(), std::vector<int>{}});
        for (Point &p : robot.getRemainingStations()) {
            plans.at(robot.getName()).emplace_back(p.getId());
        }
    }

    //Get the information about the whereabouts of the currentRobot.
    for(auto& robot: robots){
        if(robot.atPoint()){
            std::string robot_name = robot.getName();
            current_plans_and_locations.push_back(robot_at_point{robot_name, plans.at(robot_name),robot.getLatestPoint().getId()});
        }
        else {
            int latest_point = robot.getLatestPoint().getId();
            int current_target = robot.getCurrentTarget()->getId();

            //Get time spend on edge:
            int time_traveled = (int)get_distance_to_latest_point(robot);

            std::string robot_name = robot.getName();
            current_plans_and_locations.push_back(robot_moving{robot_name, plans.at(robot_name),
                                                     latest_point, current_target, time_traveled});
        }
    }

    return current_plans_and_locations;
}

std::string constructUppaalModel(const std::vector<Robot> &robots, Robot &currentRobot, bool stations){

    std::vector<abs_robot_info> current_plans_and_locations = get_robot_plans_and_positions(robots, currentRobot);


    return "";
}

std::string numOfStations(const Map_Structure &map_structure){
    return std::to_string(map_structure.stationIDs.size() + map_structure.endStationIDs.size());
}

std::string numOfPoints(const Map_Structure &map_structure){
    return std::to_string(map_structure.stationIDs.size() +
                          map_structure.endStationIDs.size() +
                          map_structure.waypointsIDs.size());
}

int number_of_robots(const Map_Structure& map_structure){
    return (int)map_structure.Robots.size();
}

std::vector<int> get_end_stations(const Map_Structure &map_structure){
    return map_structure.endStationIDs;
}

// @todo: is it on purpose that the float is made into an int in the distance matrix?
std::vector<std::vector<int>> get_distances(const Map_Structure& map_structure){
    auto sizeLines = (unsigned)sqrt(map_structure.lines.size()); //@todo: Make 2-dimentional to begin with.

    std::vector<std::vector<int>> waypointsDistances{sizeLines, std::vector<int>()};

    const std::vector<Line>& lines = map_structure.get_lines();

    int k = -1;
    for (long unsigned i = 0; i < lines.size(); i++) {
        if (i % sizeLines == 0)
            k++;
        else
            waypointsDistances[k].push_back((int)lines[i].GetDistance());
    }

    return waypointsDistances;
}

void configure_static_settings_of_Uppaal_model(Map_Structure& map_structure){
    std::ifstream blueprint{std::string{std::filesystem::current_path()} + "/station_planning_blueprint.xml"};
    std::ofstream partial_blueprint{std::string{std::filesystem::current_path()} + "/partial_blueprint.xml"};


    std::string line;
    while(std::getline(blueprint, line)){
        auto pos = line.find("#MAX_STATIONS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#MAX_STATIONS#"}.size(), numOfStations(map_structure));
        }

        pos = line.find("#OTHER ROBOTS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#OTHER_ROBOTS#"}.size(), std::to_string(number_of_robots(map_structure) - 1));
        }

        std::string matrix = getStationDistanceMatrix(map_structure);

        pos = line.find("#DISTANCE_MATRIX#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#DISTANCE_MATRIX#"}.size(), matrix);
        }

        partial_blueprint << line << std::endl;

    }



    //exit(0);

    return;
};

std::string getStationDistanceMatrix(Map_Structure &map_structure){
    // Map_structure calculates all the fastests paths between all the stations.
    std::vector<std::vector<float>> distance_matrix = map_structure.floydShortestOfStations();

    return formatMatrix(distance_matrix);
}


std::string combineMatrixLines(const std::vector<std::vector<std::string>> &distance_values){
    // Temporary results of the lines in the matrix on string-form.
    std::vector<std::string> waited_matrix{};

    // Gets all the line with correct post-/prefix and delimitor.
    for(auto& dist_line : distance_values){
        std::string line = element_joiner(dist_line, ", ", "{", "}");
        waited_matrix.emplace_back(line);
    }

    std::string final_matrix = element_joiner(waited_matrix, ",\n", "{\n", "\n}");

    return final_matrix;
}

std::size_t numOfOtherActiveRobots(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>>& otherBots){
    std::size_t robots_with_jobs = 0;

    for(auto& bot : otherBots){
        if (bot.get().hasJob()){
            robots_with_jobs++;
        }
    }

    return robots_with_jobs;
}

// Gets the distances between all stations and the point given as argunent
std::string get_expanded_distance_matrix(Map_Structure &map_structure, const Point &point){
    // Copies the full distance matrix and the short distance between stations.
    const std::vector<std::vector<float>>& fullDistMatrix = map_structure.getShortestDistanceMatrix();
    std::vector<std::vector<float>> newDistMatrix = map_structure.floydShortestOfStations();

    int p_id = point.getId();

    // Adds the distance from stations to point.
    for(std::size_t i = 0; i < newDistMatrix.size(); i++){
        newDistMatrix[i].push_back(fullDistMatrix[i][p_id]);
    }

    // Adding distances from point to stations ti the matrix.
    std::vector<float> pointToStations(newDistMatrix.size() + 1, 0);
    for(std::size_t i = 0; i < newDistMatrix.size(); i++){
        pointToStations[i] = fullDistMatrix[p_id][i];
    }
    newDistMatrix.push_back(pointToStations);

    return formatMatrix(newDistMatrix);
}

std::string format_order(int numOfStations, std::set<int> order){
    std::vector<int> verbatimOrder = convertIDsToBools<std::set<int>>(numOfStations, std::move(order));

    std::string formatted_order = element_joiner(verbatimOrder, ", ", "{", "}");

    return formatted_order;
}

std::string format_endstations(int numOfStations, std::vector<int> endstationIDs){
    std::vector<int> verbatimOrder = convertIDsToBools(numOfStations, std::move(endstationIDs));

    std::string formatted_endstations = element_joiner(verbatimOrder, ", ", "{", "}");

    return formatted_endstations;
}

std::vector<std::vector<float>> getDistanceMatrix(Map_Structure &map_structure){
    int sizeLines = std::sqrt(map_structure.lines.size()); //@todo: Make 2-dimentional to begin with.
    std::vector<std::vector<float>> waypointsDistances(sizeLines, std::vector<float>());

    int k = -1;
    for (long unsigned i = 0; i < map_structure.lines.size(); i++) {
        if (i % sizeLines == 0)
            k++;
        if(map_structure.lines[i].GetDistance()==-1)
            waypointsDistances[k].push_back(0);
        else
            waypointsDistances[k].push_back(map_structure.lines[i].GetDistance()/(double)VELOCITY*100);
    }

    return waypointsDistances;
}

std::string formatStationOrderLenghts(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots){
    std::vector<unsigned> orderLenghts{};

    for(auto& bot: otherBots){
        if(bot.get().hasJob())
            orderLenghts.push_back(bot.get().sizeOfStationPlan());
    }

    return element_joiner(orderLenghts, ", ", "{", "}");
}

std::string formatWaypointOrderLenghts(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots){
    std::vector<unsigned> orderLenghts{};

    for(auto& bot: otherBots){
        if(bot.get().hasJob())
            orderLenghts.push_back(1);
    }

    return element_joiner(orderLenghts, ", ", "{", "}");
}

std::string formatOrthersStartLocs(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots){
    std::vector<int> otherLocations{};

    for(auto& bot: otherBots) {
        //@todo: Must be fixed when they can work at stations.
        if (bot.get().hasJob()){
            //otherLocations.push_back(bot.get().getLastLocation());
            otherLocations.push_back(0);
        }
    }

    return element_joiner(otherLocations, ", ", "{", "}");
}

std::string formatOrtherWaypointStartLocs(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots){
    std::vector<int> otherLocations{};

    for(auto& bot: otherBots) {
        if (bot.get().hasJob()){
            otherLocations.push_back(bot.get().getLastLocation());
        }
    }

    return element_joiner(otherLocations, ", ", "{", "}");
}

std::string formatOtherStationPlan(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots, int numOfStations){
    std::vector<std::vector<int>> plans{};

    for(auto& bot: otherBots){
        if(bot.get().hasJob()) {
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

std::string formatOtherWaypointPlan(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots, int numOfStations){
    std::vector<std::vector<int>> plans{};

    for(auto& bot: otherBots){
        if(bot.get().hasJob()) {
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

std::string formatOtherOrders(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots, int numOfStations){
    std::vector<std::vector<int>> orders{};

    for(auto& bot: otherBots){
        if(bot.get().hasJob()) {
            std::vector<int> order = convertIDsToBools(numOfStations, bot.get().getOrder());
            orders.push_back(order);
        }
    }

    return formatMatrix(orders);
}

std::string formatOtherWaypointOrders(const std::vector<std::reference_wrapper<SingleThreadUppaalBot>> &otherBots, int numOfStations){
    std::vector<std::vector<int>> orders{};

    for(auto& bot: otherBots){
        if(bot.get().hasJob()) {
            std::vector<int> order = convertIDsToBools(numOfStations, std::vector<int>(1, bot.get().getNextStation()));
            orders.push_back(order);
        }
    }

    return formatMatrix(orders);
}

std::string format_query(unsigned numOfPoint){
    std::vector<int> points(numOfPoint);
    std::iota(std::begin(points), std::end(points), 0);

    return element_joiner(points, "],\nvisited[", "visited[", "]\n");
}