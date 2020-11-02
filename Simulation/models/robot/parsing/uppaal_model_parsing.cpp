//
// Created by martin on 20/10/20.
//

#include "uppaal_model_parsing.h"
#include "argos_wrapper/argos_wrapper.hpp"
#include "models/map/line.hpp"


std::vector<abs_robot_info> get_robot_plans_and_positions(std::vector<Robot> &robots, Robot &currentRobot){
    // What we need from the other robots are:
    // - Their plan
    // - Current location: either point or edge including time spend on the edge.
    std::map<std::string, std::vector<int>> plans{};
    std::vector<abs_robot_info> current_plans_and_locations{};

    // This gets the plans of the robots.
    for(auto& robot: robots){
        if(robot != currentRobot) {
            plans.insert({robot.getfootBot()->GetId(), std::vector<int>{}});
            for (Point &p : robot.getRemainingStations()) {
                plans.at(robot.getfootBot()->GetId()).emplace_back(p.getId());
            }
        }
    }

    //Get the information about the whereabouts of the currentRobot.
    for(auto& robot: robots){
        if(robot.atPoint()){
            std::string robot_name = robot.getfootBot()->GetId();
            current_plans_and_locations.push_back(robot_at_point{robot_name, plans.at(robot_name),robot.getLatestPoint().getId()});
        }
        else{

            int latest_point = robot.getLatestPoint().getId();
            int current_target = robot.getCurrentTarget()->getId();

            //Get time spend on edge:
            int time_traveled = (int)get_distance_to_latest_point(robot);

            std::string robot_name = robot.getfootBot()->GetId();
            current_plans_and_locations.push_back(robot_moving{robot_name, plans.at(robot_name),
                                                     latest_point, current_target, time_traveled});
        }
    }

    return current_plans_and_locations;
}

std::string constructUppaalModel(std::vector<Robot> &robots, Robot &currentRobot, bool stations){

    std::vector<abs_robot_info> current_plans_and_locations = get_robot_plans_and_positions(robots, currentRobot);


    return "";
}

int number_of_stations(const Map_Structure &map_structure){
    return (int)map_structure.stationIDs.size();
}

int number_of_waypoints(const Map_Structure &map_structure){
    return (int)map_structure.waypointsIDs.size();
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