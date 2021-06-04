#include "CTrajectoryQTUserFunctions.h"

CTrajectoryQTUserFunctions::CTrajectoryQTUserFunctions()
    : m_cTrajLF(dynamic_cast<CTrajectoryLoopFunctions &>(
                    argos::CSimulator::GetInstance().GetLoopFunctions())) {}



void CTrajectoryQTUserFunctions::KeyPressed(QKeyEvent* pc_event) {
    GetQTOpenGLWidget().KeyPressed(pc_event);
}


void CTrajectoryQTUserFunctions::draw(std::tuple<float, float, float, float> coordinates) {
    glBegin(GL_LINES);
    glVertex2f(std::get<0>(coordinates), std::get<1>(coordinates));
    glVertex2f(std::get<2>(coordinates), std::get<3>(coordinates));
    glEnd();
}

void CTrajectoryQTUserFunctions::draw(std::vector<std::tuple<float, float, float, float>> coordinates) {
    for (auto& c : coordinates) {
        draw(c);
    }
}

void CTrajectoryQTUserFunctions::DrawInWorld() {
    DrawText(argos::CVector3(1,0,1), "X=1", argos::CColor::ORANGE);
    DrawText(argos::CVector3(-1,0,1), "X=-1", argos::CColor::ORANGE);
    DrawText(argos::CVector3(0,1,1), "Y=1", argos::CColor::ORANGE);
    DrawText(argos::CVector3(0,-1,1), "Y=-1", argos::CColor::ORANGE);
    for (auto element : sMap.points) {
        if (element.getName().at(0) != 'o')
            DrawText(element, element.getName() + "/" + std::to_string(element.getId()));
        else
            DrawText(element, std::to_string(element.getId()));
        element.SetZ(0.1);
        DrawPoint(element, argos::CColor::BLACK, 5);
    }

    //DrawText(argos::CVector3(0.0, 0.0, 0.3), "0");

    for (auto &element : sMap.boxes) {
        draw(element.getCoordinates());
    }

    for (auto &element : sMap.lines) {
        //if (element.GetDistance() != -1 && element.Geta().getName().at(0) != 'S' && element.Getb().getName().at(0) != 'S')
        if (element.GetDistance() != -1)
            draw(element.getCoordinates());

    }

    // Draws robots
    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        DrawText(pcBot->GetEmbodiedEntity().GetOriginAnchor().Position, pcBot->GetId());
    }

}

/****************************************/
/****************************************/
REGISTER_QTOPENGL_USER_FUNCTIONS(CTrajectoryQTUserFunctions, "CTrajectoryQTUserFunctions")
