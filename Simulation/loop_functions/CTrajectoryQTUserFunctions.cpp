#include "CTrajectoryQTUserFunctions.h"

CTrajectoryQTUserFunctions::CTrajectoryQTUserFunctions()
    : m_cTrajLF(dynamic_cast<CTrajectoryLoopFunctions &>(
                    argos::CSimulator::GetInstance().GetLoopFunctions())) {}



void CTrajectoryQTUserFunctions::KeyPressed(QKeyEvent* pc_event) {
    switch(pc_event->key()) {
        case Qt::Key_I:

            for(auto& robot: Map_Structure::get_instance().Robots){
                argos::LOG<<"Robot ID: "<< robot.getfootBot()->GetId() <<std::endl;
                argos::LOG<<"Current Location: "<< robot.getLatestPoint().getName() <<std::endl;
                if(!robot.getRemainingWaypoints().empty()){
                    argos::LOG<<"My current Location: "<< robot.getRemainingWaypoints().front().getName() <<std::endl;
                    argos::LOG<<"My remaining stations: " <<std::endl;
                    for(auto& station: robot.getRemainingStations()){
                        argos::LOG<<" "<<station.getName()<<std::endl;
                    }
                }
                else
                    argos::LOG<<"My target Location: Initial"<<std::endl;
                drawExpected= true;


            }break;
        default:
            /* Unknown key */
            GetQTOpenGLWidget().KeyPressed(pc_event);
            break;
    }
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
        DrawText(element, element.getName());
        element.SetZ(0.1);
        DrawPoint(element, argos::CColor::BLACK, 5);
    }

    for (auto &element : sMap.boxes) {
        draw(element.getCoordinates());
    }

    for (auto &element : sMap.lines) {
        if (element.GetDistance() != -1)
            draw(element.getCoordinates());
    }

    for (auto &robot : sMap.Robots) {
        DrawText(robot.getfootBot()->GetEmbodiedEntity().GetOriginAnchor().Position, robot.getfootBot()->GetId());
    }
    DrawText(argos::CVector3(0.0, 0.0, 0.3), "0");
    if(drawExpected){
        for(auto& robot: Map_Structure::get_instance().Robots.back().getOtherRobotsEstimates()){
            argos::CVector3 position = robot->currPosition;
            position.SetZ(0.1);
            Robot otherRobot= sMap.Robots[sMap.getRobotIdByName(robot->id)];
            for(long unsigned i = 0; i < robot->allPassedPoints.size(); i++){
                if(i+1 != robot->allPassedPoints.size())DrawRay(argos::CRay3(robot->allPassedPoints[i],robot->allPassedPoints[i+1]), argos::CColor::RED, 3);
                if(i+1 == robot->allPassedPoints.size()) DrawRay(argos::CRay3(robot->allPassedPoints[i],position), argos::CColor::RED, 3);
            }
            DrawPoint(position, argos::CColor::RED, 10);
            DrawText(position, robot->id, argos::CColor::ORANGE);
        }
    }

}

/****************************************/
/****************************************/
REGISTER_QTOPENGL_USER_FUNCTIONS(CTrajectoryQTUserFunctions, "CTrajectoryQTUserFunctions")
