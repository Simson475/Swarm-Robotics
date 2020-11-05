//
// Created by martin on 20/10/20.
//

#include "uppaal_model_parsing.h"
#include "argos_wrapper/argos_wrapper.hpp"
#include "models/map/line.hpp"

#include <iostream>
#include <fstream>
#include <filesystem>

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

std::string number_of_stations(const Map_Structure &map_structure){
    return std::to_string(map_structure.stationIDs.size() + map_structure.endStationIDs.size());
}

int number_of_waypoints(const Map_Structure &map_structure){
    return (int)map_structure.waypointsIDs.size();
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
            line.replace(pos, std::string{"#MAX_STATIONS#"}.size(), number_of_stations(map_structure));
        }

        pos = line.find("#OTHER ROBOTS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#OTHER ROBOTS#"}.size(), std::to_string(number_of_robots(map_structure) - 1));
        }

        pos = line.find("#DISTANCE_MATRIX#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#DISTANCE_MATRIX#"}.size(), get_distance_matrix(map_structure));
        }

        partial_blueprint << line << std::endl;

    }

    exit(0);

    return;
};

std::string get_distance_matrix(Map_Structure& map_structure){
    // Map_structure calculates all the fastests paths between all the stations.
    std::vector<std::vector<float>> distance_matrix = map_structure.floydShortestOfStations();

    std::vector<std::vector<std::string>> distance_values_str = format_distance_values(distance_matrix);

    return format_distance_matrix(distance_values_str);
}

std::vector<std::vector<std::string>> format_distance_values(const std::vector<std::vector<float>>& dist_matrix){
    std::vector<std::vector<std::string>> distance_matrix_str{};

    for(auto& line : dist_matrix){
        std::vector<std::string> formatted_numbers{};
        for(auto& number : line) {
            formatted_numbers.push_back(std::to_string(number));
        }
        distance_matrix_str.push_back(formatted_numbers);
    }

    return distance_matrix_str;
}

std::string format_distance_matrix(const std::vector<std::vector<std::string>>& distance_values){
    // Temporary results of the lines in the matrix on string-form.
    std::vector<std::string> waited_matrix{};

    // Gets all the line with correct post-/prefix and delimitor.
    for(auto& dist_line : distance_values){
        std::stringstream line{};
        line << "{";
        std::copy(dist_line.begin(),
                  dist_line.end(),
                  std::experimental::make_ostream_joiner(line, ", "));
        line << "}";
        waited_matrix.emplace_back(line.str());
    }

    // Connects all the line with correct post-/prefix and delimitor.
    std::stringstream final_matrix{};
    final_matrix << "{\n";
    std::copy(waited_matrix.begin(),
              waited_matrix.end(),
              std::experimental::make_ostream_joiner(final_matrix, ",\n"));
    final_matrix << "\n}";

    return final_matrix.str();
}