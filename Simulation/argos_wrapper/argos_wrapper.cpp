//
// Created by martin on 29/10/20.
//

#include "argos_wrapper.hpp"

#include "argos3/core/utility/math/vector3.h"
#include "argos3/plugins/robots/foot-bot/simulator/footbot_entity.h"
#include "argos3/core/simulator/loop_functions.h"


double getDistanceToNextPoint(const argos::CCI_Controller& controller, Map_Structure map_structure, int nextPoint){
    auto& footbot = dynamic_cast<argos::CFootBotEntity&>(argos::CLoopFunctions().GetSpace().GetEntity(controller.GetId()));

    auto position = footbot.GetEmbodiedEntity().GetOriginAnchor().Position;
    return argos::Distance(position, map_structure.getPointByID(nextPoint));
}
