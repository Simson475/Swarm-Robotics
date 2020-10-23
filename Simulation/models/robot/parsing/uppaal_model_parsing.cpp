//
// Created by martin on 20/10/20.
//

#include "uppaal_model_parsing.h"


std::string constructUppaalModel(std::vector<Robot> &robots, Robot &currentRobot, bool stations){
    // What we need from the other robots are:
    // - Their plan
    // - Current location: either point or edge including time spend on the edge.
    std::map<std::string, std::vector<int>> plans{};
    std::vector<std::pair<bool,int>> current_locations{};

    // This gets the plans of the robots.
    for(auto& other_robot: robots){
        if(other_robot != currentRobot){
            plans.insert({other_robot.getfootBot()->GetId(), std::vector<int>{}});
            for(Point& p : other_robot.getRemainingStations()){
                plans.at(other_robot.getfootBot()->GetId()).emplace_back(p.getId());
            }
        }
    }

    //Get the information about the whereabouts of the currentRobot.
    for(auto& other_robot: robots){
        if(other_robot.atPoint()){
            current_locations.push_back(std::make_pair(true, other_robot.getLatestPoint().getId()));
        }
        else{
            // Edges does not have a unique ID. (We can give it on instantiation)
            // Last point through Robot::getLatestPoint() and latest through Robot::getCurrentTarget().
            // Map_Structure has a vector of lines.
            current_locations.push_back(std::make_pair(false, other_robot.getLatestPoint().getId()));
        }
    }

    //Get the

    //Get other robots plans.


    return "";
}