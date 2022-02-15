//
// Created by martin on 29/10/20.
//

#include "argos_wrapper.hpp"

#include "argos3/core/utility/math/vector3.h"
#include "argos3/plugins/robots/foot-bot/simulator/footbot_entity.h"
#include "argos3/core/simulator/loop_functions.h"

/**
 * Gets the distance from the robot the controller controls to the next point
 * 
 * @param controller argos::CCI_Controller&
 * @param map_structure Map_Structure&
 * @param nextPointId int
 * 
 * @return double
*/
double getDistanceToNextPoint(const argos::CCI_Controller& controller, Map_Structure& map_structure, int nextPointId){
    auto& footbot = dynamic_cast<argos::CFootBotEntity&>(argos::CLoopFunctions().GetSpace().GetEntity(controller.GetId()));

    auto position = footbot.GetEmbodiedEntity().GetOriginAnchor().Position;
    return argos::Distance(position, map_structure.getPointByID(nextPointId));
}

/**
 * Gets the euclidian path distance
 * 
 * @param map_structure Map_Structure&
 * @param pointIds std::vector<int>
 * 
 * @return double
*/
double getDistanceBetweenPoints(Map_Structure& map_structure, std::vector<int> pointIds){
    double path_distance = 0;

    for(unsigned i = 1; i < pointIds.size(); i++){
        path_distance += argos::Distance(
            map_structure.getPointByID(pointIds[i-1]),
            map_structure.getPointByID(pointIds[i]));
    }

    return path_distance;
}

/**
 * Returns the CSimulator instance's space's clock.
 * 
 * @return int
*/
int getLogicalTime(){
    return argos::CSimulator::GetInstance().GetSpace().GetSimulationClock();
}