#ifndef CTrajectoryQTUserFunctions_H
#define CTrajectoryQTUserFunctions_H


#include "models/map/map_structure.hpp"
#include "CTrajectoryLoopFunctions.h"


#include "argos3/core/control_interface/ci_controller.h"
#include "argos3/core/simulator/loop_functions.h"
#include "argos3/plugins/robots/foot-bot/simulator/footbot_entity.h"
#include "argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h"
#include "argos3/plugins/simulator/entities/box_entity.h"
#include "argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h"
#include "QKeyEvent"
#include "nlohmann/json.hpp"


class CTrajectoryQTUserFunctions : public argos::CQTOpenGLUserFunctions {

public:
    CTrajectoryQTUserFunctions();
    virtual ~CTrajectoryQTUserFunctions() {}

    // method which is responsible of displaying everything in GUI
    virtual void DrawInWorld();

    //captures key presses during the simulation
    virtual void KeyPressed(QKeyEvent* pc_event);

private:
    CTrajectoryLoopFunctions &m_cTrajLF;
    Map_Structure &sMap = Map_Structure::get_instance();
    bool drawExpected = false;
};

#endif
