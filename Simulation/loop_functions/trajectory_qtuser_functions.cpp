#include "trajectory_qtuser_functions.h"
#include "trajectory_loop_functions.h"
#include <bits/stdc++.h>
#include <cctype>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

// performs all the actions only once, upon start of the program.
#define newJobs true

/* method used for testing the shortest paths between points
void print(std::vector<std::vector<Line>> next) {
  std::cout << "(pair, path)" << std::endl;
  const auto size = next.size();
  for (auto i = 0; i < size; ++i) {
    for (auto j = 0; j < size; ++j) {
      if (i != j) {
        auto u = i;
        auto v = j;
        std::cout << "(" << next[i][j].Geta().getName() << " -> " << v << ", ";
        std::stringstream path;
        path << next[i][j].Geta().getName();
        do {
          
          u = next[u][v].Getb().getId();
          path << " -> " << u;
        } while (u != v);
        std::cout << path.str() << ")" << std::endl;
      }
    }
  }
}*/

CTrajectoryQTUserFunctions::CTrajectoryQTUserFunctions()
    : m_cTrajLF(dynamic_cast<CTrajectoryLoopFunctions &>(
          CSimulator::GetInstance().GetLoopFunctions())) {
  if(newJobs)
    generateJobs(); // call this for generating new list of jobs
  sMap.initializeStations("experiment/scene2/"); 
  sMap.initializeJobs("experiment/scene2/");
  sMap.collectAllWayPoints();
  sMap.createFolderForEachRobot("experiment/scene2/");
  sMap.setAllPossibleLines();
  sMap.createStaticJSON("experiment/scene2/");
}
void CTrajectoryQTUserFunctions::generateJobs() {
  //************creates random jobs in json file************
std::random_device rd;  // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<> distr(2, 11);   // define the range of stations ids
  std::uniform_int_distribution<> distrEnd(3, 4); // define the range of how many stations to visit
  std::uniform_int_distribution<> endPoints(0, 1);//define end points ids
nlohmann::json mainJsonObj;
  for (auto i = 0; i < 100; i++){
    nlohmann::json jsonObj;
    jsonObj["job_id"] = i;
    vector<nlohmann::json> stationsToVisit;
    int amountPickups = distrEnd(eng);
    for (auto j = 0; j < amountPickups ;j++) {
      int temp = distr(eng);
      bool check = true;
      for(auto& station: stationsToVisit){
        if(station == temp){ check = false;break;}}
      if(check)stationsToVisit.push_back(temp);
    }
    stationsToVisit.push_back(endPoints(eng));
    jsonObj["job"] = stationsToVisit;
      mainJsonObj.push_back(jsonObj);
  }
  std::ofstream out("experiment/scene2/jobs.json");
  out << std::setw(4) << mainJsonObj;
}
void CTrajectoryQTUserFunctions::KeyPressed(QKeyEvent* pc_event) {
   switch(pc_event->key()) {
      case Qt::Key_I:

         for(auto& robot: Map_Structure::get_instance().Robots){
           argos::LOG<<"Robot ID: "<< robot.getfootBot()->GetId() <<std::endl;
           argos::LOG<<"Current Location: "<< robot.getCurrentID().getName() <<std::endl;
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

void CTrajectoryQTUserFunctions::DrawInWorld() {
   DrawText(CVector3(1,0,1), "X=1", CColor::ORANGE);
   DrawText(CVector3(-1,0,1), "X=-1", CColor::ORANGE);
      DrawText(CVector3(0,1,1), "Y=1", CColor::ORANGE);
   DrawText(CVector3(0,-1,1), "Y=-1", CColor::ORANGE);
  for (auto element : sMap.points) {
    DrawText(element, element.getName());
    element.SetZ(0.1);
    DrawPoint(element,CColor::BLACK, 5);
  }
  for (auto &element : sMap.boxes) {
    element.draw();
  }
  for (auto &element : sMap.lines) {
    if (element.GetDistance() != -1)
      element.draw();
  }
    for (auto &robot : sMap.Robots) {
    DrawText(robot.getfootBot()->GetEmbodiedEntity().GetOriginAnchor().Position, robot.getfootBot()->GetId());
  }
  DrawText(CVector3(0.0, 0.0, 0.3), "0");
   if(drawExpected){
      for(auto& robot: Map_Structure::get_instance().Robots.back().getOtherRobotsEstimates()){
          argos::CVector3 position = robot->currPosition;
          position.SetZ(0.1);
          Robot otherRobot= sMap.Robots[sMap.getRobotById(robot->id)];
          for(int i = 0; i < robot->allPassedPoints.size(); i++){
            if(i+1 != robot->allPassedPoints.size())DrawRay(CRay3(robot->allPassedPoints[i],robot->allPassedPoints[i+1]),CColor::RED, 3);
            if(i+1 == robot->allPassedPoints.size()) DrawRay(CRay3(robot->allPassedPoints[i],position),CColor::RED, 3);
          }
          DrawPoint(position,CColor::RED, 10);
          DrawText(position, robot->id,CColor::ORANGE);
      } 
   }

} 

/****************************************/
/****************************************/
REGISTER_QTOPENGL_USER_FUNCTIONS(CTrajectoryQTUserFunctions,
                                 "trajectory_qtuser_functions")
