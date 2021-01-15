//
// Created by martin on 29/10/20.
//

#include "argos_wrapper.hpp"

#include "argos3/core/utility/math/vector3.h"
#include "argos3/plugins/robots/foot-bot/simulator/footbot_entity.h"
#include "argos3/core/simulator/loop_functions.h"


double getDistanceToNextPoint(const argos::CCI_Controller& controller, Map_Structure& map_structure, int nextPoint){
    auto& footbot = dynamic_cast<argos::CFootBotEntity&>(argos::CLoopFunctions().GetSpace().GetEntity(controller.GetId()));

    auto position = footbot.GetEmbodiedEntity().GetOriginAnchor().Position;
    return argos::Distance(position, map_structure.getPointByID(nextPoint));
}

double getDistanceBetweenPoints(Map_Structure& map_structure, std::vector<int> points){
    double path_distance = 0;

    for(unsigned i = 1; i < points.size(); i++){
        path_distance += argos::Distance(
            map_structure.getPointByID(points[i-1]),
            map_structure.getPointByID(points[i]));
    }

    return path_distance;
}

int getLogicalTime(){
    return argos::CSimulator::GetInstance().GetSpace().GetSimulationClock();
}