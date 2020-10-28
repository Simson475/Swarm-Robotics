//
// Created by martin on 20/10/20.
//

#include "uppaal_model_parsing.h"
#include "argos3/core/utility/math/vector3.h"


std::string constructUppaalModel(std::vector<Robot> &robots, Robot &currentRobot, bool stations){
    // What we need from the other robots are:
    // - Their plan
    // - Current location: either point or edge including time spend on the edge.
    std::map<std::string, std::vector<int>> plans{};
    std::vector<abs_robot_info> current_locations{};

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
    for(auto& robot: robots){
        if(robot.atPoint()){
            std::string robot_name = robot.getfootBot()->GetId();
            current_locations.push_back(robot_at_point{robot_name, plans.at(robot_name),0});
        }
        else{
            // Edges does not have a unique ID. (We can give it on instantiation)
            // Last point through Robot::getLatestPoint() and latest through Robot::getCurrentTarget().
            // Map_Structure has a vector of lines.
            int latest_point = robot.getLatestPoint().getId();
            int current_target = robot.getCurrentTarget()->getId();

            //Get time spend:
            argos::CVector3 current_position = robot.getfootBot()->GetEmbodiedEntity().GetOriginAnchor().Position;
            int time_traveled = (int)std::round(argos::Distance(current_position, robot.getLatestPoint()));

            std::string robot_name = robot.getfootBot()->GetId();
            current_locations.push_back(robot_moving{robot_name, plans.at(robot_name),
                                                     latest_point, current_target, time_traveled});
        }
    }

    //Get the time spend on an edge:
    // robot.getLatestPoint()
    // argos::CVector3 currPosition = r.getfootBot()->GetEmbodiedEntity().GetOriginAnchor().Position;
    // argos::Distance(currPosition, robot.getLatestPoint())

    //Get other robots plans.


    return "";
}