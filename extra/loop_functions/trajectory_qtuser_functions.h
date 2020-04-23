#ifndef TRAJECTORY_QTUSER_FUNCTIONS_H
#define TRAJECTORY_QTUSER_FUNCTIONS_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <QKeyEvent>
#include <json.hpp>

#include "map_structure.h"

using namespace argos;

class CTrajectoryLoopFunctions;

class CTrajectoryQTUserFunctions : public CQTOpenGLUserFunctions {

public:
  CTrajectoryQTUserFunctions();

  virtual ~CTrajectoryQTUserFunctions() {}

  virtual void DrawInWorld();

  argos::Real velocity;

  vector<vector<float>> floydShortest(int amountOfStations);
  void initializeStations(std::string path);
  void createFolderForEachRobot(std::string path);
  void collectAllWayPoints();
  void setAllPossibleLines();
  void eliminateBadLines();
  void createStaticJSON();
  void generateJobs();
  void initializeJobs(std::string path);
  virtual void KeyPressed(QKeyEvent* pc_event);

private:
  CTrajectoryLoopFunctions &m_cTrajLF;
  Map_Structure &sMap = Map_Structure::get_instance();
  CCI_PositioningSensor *m_pcPosition;
  bool drawExpected = false;
};

#endif
