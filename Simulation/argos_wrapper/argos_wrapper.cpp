//
// Created by martin on 29/10/20.
//

#include "argos_wrapper.hpp"


double get_distance_to_latest_point(const Robot& robot) {
    if (robot.atPoint())
        return 0.0;

    argos::CVector3 current_position = robot.getfootBot()->GetEmbodiedEntity().GetOriginAnchor().Position;
    return std::round(argos::Distance(current_position, robot.getLatestPoint()));
}
