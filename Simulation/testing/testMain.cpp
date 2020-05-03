#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "json.hpp"
#include <cmath>
#include <iostream>
#include <dirent.h>
#include <sys/types.h>
#include <experimental/filesystem>
#include "../loop_functions/map_elements.h"
#include "../loop_functions/robot.h"
#include "../loop_functions/map_structure.h"
#include "../loop_functions/trajectory_qtuser_functions.h"
#include <argos3/core/utility/configuration/argos_configuration.h>

using namespace std;
using namespace argos;


struct Singleton{
private:
    static Singleton * instance;
    Singleton(){
        static argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
        cSimulator.SetExperimentFileName("../experiment/scene2/trajectory.argos");
        cSimulator.LoadExperiment();
    }
    Singleton(const Singleton & source){}
    Singleton(Singleton && source){}
public:
    static Singleton &GetInstance(){
      static Singleton instance;
      return instance;
    }
};


TEST_CASE("creation of point", "[base elements]"){
    Point p = Point(1,2,0,Type::station,std::string("P01"));
    REQUIRE( p.getId() == 0 );
    REQUIRE( p.GetX() == 1 );
    REQUIRE( p.GetY() == 2 );
    REQUIRE( p.getType() == Type::station );
}
TEST_CASE("creation of line", "[base elements]"){
  SECTION( "line of distance 0 " ) {
    Point p1 = Point(1,2,0,Type::station,std::string("P01"));
    Point p2 = Point(1,2,0,Type::station,std::string("P02"));
    Line l = Line(&p1, &p2);
    REQUIRE( l.getTime() == 0  );
    REQUIRE( l.Geta().getName() == "P01" );
    REQUIRE( l.Getb().getName() == "P02" );
  }
  SECTION( "line with distance between 2 points" ) {
    Point p1 = Point(1,2,0,Type::station,std::string("P02"));
    Point p2 = Point(1,4,0,Type::station,std::string("P03"));
    Line l = Line(&p1, &p2);
    REQUIRE( l.GetDistance() == 2);
    l.setFailureline();
    REQUIRE( l.GetDistance() == -1 );
    REQUIRE( l.getFloydTime() == INF );
  }
}
TEST_CASE("creation of box", "[base elements]"){
  SECTION( "basic box" ) {
    Box b = Box("box1", CVector3(3,3,0), CVector3(4,6,0.4));
    b.setBoxCorner();
    for(auto i = 0; i < 4; i++){
      float actual = roundf( argos::Distance(b.getBoxLine(i).Geta(), b.getVCorner(i)) * 100) / 100;
      float expected = roundf( sqrt(pow(offset,2)+ pow(offset,2)) * 100) / 100;
      REQUIRE( actual==expected);
      REQUIRE(b.getVCorner(i).getType() == Type::via);
      REQUIRE(b.getBoxLine(i).Getb().getType() == Type::realCorner);
    }
  }
}

TEST_CASE("creation of robot", "[base elements]"){
  Singleton::GetInstance();
  Point p = Point(1,2,0,Type::station,std::string("FB20.Initial"));

  CFootBotEntity* fb =new CFootBotEntity("FB20","fdc",p.getCVector());
  Robot r = Robot(fb, &p);
  SECTION( "linking to footBot" ) {

    REQUIRE(r.getfootBot()->GetId() == "FB20");
  }
  SECTION( "initial location " ) {
    p.resetIdCount();
    REQUIRE(r.getInitialLoc().getName() == "FB20.Initial");
  }
}

TEST_CASE("intilization of objects", "[map init]"){
  Map_Structure &sMap = Map_Structure::get_instance();
  std::string path = "../experiment/scene2/";
  
  SECTION( "correct amount of stations parsed" ) {
    sMap.initializeStations(path);
    int amount = 0;
    for (auto& point :  Map_Structure::get_instance().points){
      if(point.getType() == 2)
        amount++;
    }
    REQUIRE(amount == 10);
  }
  SECTION( "correct amount of jobs parsed" ) {
    sMap.initializeJobs(path);
    REQUIRE(Map_Structure::get_instance().jobs.size() == 20);
  }
  SECTION( "collects correct amount of waypoints" ) {
    sMap.collectAllWayPoints();
    REQUIRE(Map_Structure::get_instance().points.size() == 23);
  }
  SECTION( "creates a folder for each robot" ) {
    system("cd ../experiment/scene2/ && find *fb* -delete"); 
    std::string path = "../experiment/scene2/";
    sMap.createFolderForEachRobot(path);
    int amount = 0;
    for (const auto & entry : std::experimental::filesystem::directory_iterator(path))
      if(entry.path().string().find("fb") != std::string::npos)
        amount++;
    REQUIRE(amount == sMap.Robots.size());
  }
  SECTION( "checks for the correct amount of projections between points" ) {
    sMap.setAllPossibleLines();
    REQUIRE(sMap.lines.size() == pow(sMap.points.size(),2));
  }
  SECTION( "check if lines cannot exist" ) {
    int size = sMap.points.size();
      REQUIRE(sMap.lines[12*size+17].GetDistance() == -1 );
     REQUIRE(sMap.lines[12*size+5].GetDistance() == -1 );
     REQUIRE(sMap.lines[12*size+6].GetDistance() == -1 );
     REQUIRE(sMap.lines[12*size+20].GetDistance() == -1 );  
     REQUIRE(sMap.lines[12*size+11].GetDistance() == -1 );  
     REQUIRE(sMap.lines[12*size+7].GetDistance() == -1 );  
     REQUIRE(sMap.lines[4*size+21].GetDistance() != -1 );// straight line interupted by other virtual corner 
     REQUIRE(sMap.lines[4*size+7].GetDistance() == -1 ); 
     REQUIRE(sMap.lines[4*size+8].GetDistance() == -1 ); 
  }
  SECTION( "creation of static json map" ) {
    REQUIRE(sMap.points.size() == 23);
    REQUIRE(sMap.lines.size() == 529);
    REQUIRE(sMap.hardLines.size() == 8);
    REQUIRE(sMap.Robots.size() == 3);
    REQUIRE(sMap.jobs.size() == 20);
    REQUIRE(sMap.stationIDs.size() == 10);
    REQUIRE(sMap.endStationIDs.size() == 2);
  /*Map_Structure &sMap = Map_Structure::get_instance();
  sMap.initializeStations("../experiment/scene2/");
  sMap.initializeJobs("../experiment/scene2/");
  sMap.collectAllWayPoints();
  sMap.createFolderForEachRobot("../experiment/scene2/");
  sMap.setAllPossibleLines();*/
  sMap.createStaticJSON(path);
  
  }
}
TEST_CASE("horizontal/vertical line", "[predicted location]"){
  Point p = Point(1,2,0,Type::station,std::string("FB21.Initial"));
  CFootBotEntity* fb =new CFootBotEntity("FB21","fdc",p.getCVector());
  Robot r = Robot(fb, &p);
  SECTION( "x positive" ) {
    Point p1 = Point(CVector3(2, 0, 0),Type::via, "Start");
    Point p2 = Point(CVector3(5, 0, 0),Type::via, "end");
    
    REQUIRE(r.getPosition(p1,p2,1.5).GetX() == 3.5);
  }
    SECTION( "x negative" ) {
    Point p1 = Point(CVector3(-2, 0, 0),Type::via, "Start");
    Point p2 = Point(CVector3(-5, 0, 0),Type::via, "end");
    
    REQUIRE(r.getPosition(p1,p2,1.5).GetX() == -3.5);
  }
    SECTION( "y positive" ) {
    Point p1 = Point(CVector3(0, 2, 0),Type::via, "Start");
    Point p2 = Point(CVector3(0, 5, 0),Type::via, "end");
    
    REQUIRE(r.getPosition(p1,p2,1.5).GetY() == 3.5);
  }
    SECTION( "y negative" ) {
    Point p1 = Point(CVector3(0, -2, 0),Type::via, "Start");
    Point p2 = Point(CVector3(0, -5, 0),Type::via, "end");
    
    REQUIRE(r.getPosition(p1,p2,1.5).GetY() == -3.5);
  }
}
TEST_CASE("diagonal line start closer to center", "[predicted location]"){
  Point p = Point(1,2,0,Type::station,std::string("FB21.Initial"));
  CFootBotEntity* fb =new CFootBotEntity("FB21","fdc",p.getCVector());
  Robot r = Robot(fb, &p);
  SECTION( "x/y positive" ) {
    Point p1 = Point(CVector3(2, 1, 0),Type::via, "Start");
    Point p2 = Point(CVector3(7, 6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,4.24);
    //REQUIRE((x < 0.1 && x > -0.1));
    REQUIRE(round(result.GetX()) == 5 );// around 3.5
    REQUIRE(round(result.GetY()) == 4 );// around 0.5
  }
    SECTION( "x negative" ) {
    Point p1 = Point(CVector3(-2, 1, 0),Type::via, "Start");
    Point p2 = Point(CVector3(-7, 6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,4.24);
    REQUIRE(round(result.GetX()) == -5 );// around 3.5
    REQUIRE(round(result.GetY()) == 4 );// around 0.5
  }
    SECTION( "y/x negative" ) {
    Point p1 = Point(CVector3(-2, -1, 0),Type::via, "Start");
    Point p2 = Point(CVector3(-7, -6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,4.24);
    REQUIRE(round(result.GetX()) == -5 );// around 3.5
    REQUIRE(round(result.GetY()) == -4 );// around 0.5
  }
    SECTION( "y negative" ) {
    Point p1 = Point(CVector3(2, -1, 0),Type::via, "Start");
    Point p2 = Point(CVector3(7, -6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,4.24);
    REQUIRE(round(result.GetX()) == 5 );// around 3.5
    REQUIRE(round(result.GetY()) == -4 );// around 0.5
  }
}


TEST_CASE("diagonal line start closer to center, reversed start/end", "[predicted location]"){
  Point p = Point(1,2,0,Type::station,std::string("FB21.Initial"));
  CFootBotEntity* fb =new CFootBotEntity("FB21","fdc",p.getCVector());
  Robot r = Robot(fb, &p);
  SECTION( "x/y positive" ) {
    Point p2 = Point(CVector3(2, 1, 0),Type::via, "Start");
    Point p1 = Point(CVector3(7, 6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,2.83);
    //REQUIRE((x < 0.1 && x > -0.1));
    REQUIRE(round(result.GetX()) == 5 );// around 3.5
    REQUIRE(round(result.GetY()) == 4 );// around 0.5
  }
    SECTION( "x negative" ) {
    Point p2 = Point(CVector3(-2, 1, 0),Type::via, "Start");
    Point p1 = Point(CVector3(-7, 6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,2.83);
    REQUIRE(round(result.GetX()) == -5 );// around 3.5
    REQUIRE(round(result.GetY()) == 4 );// around 0.5
  }
    SECTION( "y/x negative" ) {
    Point p2 = Point(CVector3(-2, -1, 0),Type::via, "Start");
    Point p1 = Point(CVector3(-7, -6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,2.83);
    REQUIRE(round(result.GetX()) == -5 );// around 3.5
    REQUIRE(round(result.GetY()) == -4 );// around 0.5
  }
    SECTION( "y negative" ) {
    Point p2 = Point(CVector3(2, -1, 0),Type::via, "Start");
    Point p1 = Point(CVector3(7, -6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,2.83);
    REQUIRE(round(result.GetX()) == 5 );// around 3.5
    REQUIRE(round(result.GetY()) == -4 );// around 0.5
  }
}



TEST_CASE("diagonal line end closer to center", "[predicted location]"){
  Point p = Point(1,2,0,Type::station,std::string("FB21.Initial"));
  CFootBotEntity* fb =new CFootBotEntity("FB21","fdc",p.getCVector());
  Robot r = Robot(fb, &p);
  SECTION( "x/y positive" ) {
    Point p1 = Point(CVector3(8, 1, 0),Type::via, "Start");
    Point p2 = Point(CVector3(3, 6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,4.24);
    //REQUIRE((x < 0.1 && x > -0.1));
    REQUIRE(round(result.GetX()) == 5 );// around 3.5
    REQUIRE(round(result.GetY()) == 4 );// around 0.5
  }
    SECTION( "x negative" ) {
    Point p1 = Point(CVector3(-8, 1, 0),Type::via, "Start");
    Point p2 = Point(CVector3(-3, 6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,4.24);
    REQUIRE(round(result.GetX()) == -5 );// around 3.5
    REQUIRE(round(result.GetY()) == 4 );// around 0.5
  }
    SECTION( "y/x negative" ) {
    Point p1 = Point(CVector3(-8, -1, 0),Type::via, "Start");
    Point p2 = Point(CVector3(-3, -6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,4.24);
    REQUIRE(round(result.GetX()) == -5 );// around 3.5
    REQUIRE(round(result.GetY()) == -4 );// around 0.5
  }
    SECTION( "y negative" ) {
    Point p1 = Point(CVector3(8, -1, 0),Type::via, "Start");
    Point p2 = Point(CVector3(3, -6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,4.24);
    REQUIRE(round(result.GetX()) == 5 );// around 3.5
    REQUIRE(round(result.GetY()) == -4 );// around 0.5
  }
}

TEST_CASE("diagonal line end closer to center, reversed start/end", "[predicted location]"){
  Point p = Point(1,2,0,Type::station,std::string("FB21.Initial"));
  CFootBotEntity* fb =new CFootBotEntity("FB21","fdc",p.getCVector());
  Robot r = Robot(fb, &p);
  SECTION( "x/y positive" ) {
    Point p2 = Point(CVector3(8, 1, 0),Type::via, "Start");
    Point p1 = Point(CVector3(3, 6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,2.83);
    //REQUIRE((x < 0.1 && x > -0.1));
    REQUIRE(round(result.GetX()) == 5 );// around 3.5
    REQUIRE(round(result.GetY()) == 4 );// around 0.5
  }
    SECTION( "x negative" ) {
    Point p2 = Point(CVector3(-8, 1, 0),Type::via, "Start");
    Point p1 = Point(CVector3(-3, 6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,2.83);
    REQUIRE(round(result.GetX()) == -5 );// around 3.5
    REQUIRE(round(result.GetY()) == 4 );// around 0.5
  }
    SECTION( "y/x negative" ) {
    Point p2 = Point(CVector3(-8, -1, 0),Type::via, "Start");
    Point p1 = Point(CVector3(-3, -6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,2.83);
    REQUIRE(round(result.GetX()) == -5 );// around 3.5
    REQUIRE(round(result.GetY()) == -4 );// around 0.5
  }
    SECTION( "y negative" ) {
    Point p2 = Point(CVector3(8, -1, 0),Type::via, "Start");
    Point p1 = Point(CVector3(3, -6, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,2.83);
    REQUIRE(round(result.GetX()) == 5 );// around 3.5
    REQUIRE(round(result.GetY()) == -4 );// around 0.5
  }
}

TEST_CASE("diagonal crossing x", "[predicted location]"){
  Point p = Point(1,2,0,Type::station,std::string("FB21.Initial"));
  CFootBotEntity* fb =new CFootBotEntity("FB21","fdc",p.getCVector());
  Robot r = Robot(fb, &p);
  SECTION( "cross x posivite" ) {
    Point p1 = Point(CVector3(8, -3, 0),Type::via, "Start");
    Point p2 = Point(CVector3(3, 2, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,4.24);
    //REQUIRE((x < 0.1 && x > -0.1));
    REQUIRE(round(result.GetX()) == 5 );// around 3.5
    REQUIRE(round(result.GetY()) == 0 );// around 0.5
  }
    SECTION( "cross x negative" ) {
    Point p1 = Point(CVector3(-1, -3, 0),Type::via, "Start");
    Point p2 = Point(CVector3(-6, 2, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,4.24);
    REQUIRE(round(result.GetX()) == -4 );// around 3.5
    REQUIRE(round(result.GetY()) == 0 );// around 0.5
  }
}

TEST_CASE("diagonal crossing y", "[predicted location]"){
  Point p = Point(1,2,0,Type::station,std::string("FB21.Initial"));
  CFootBotEntity* fb =new CFootBotEntity("FB21","fdc",p.getCVector());
  Robot r = Robot(fb, &p);
  SECTION( "cross y posivite" ) {
    Point p1 = Point(CVector3(1, 1, 0),Type::via, "Start");
    Point p2 = Point(CVector3(-5, 4, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,2.24);
    //REQUIRE((x < 0.1 && x > -0.1));
    REQUIRE(round(result.GetX()) == -1 );// around 3.5
    REQUIRE(round(result.GetY()) == 2 );// around 0.5
  }
  SECTION( "cross y negative 1" ) {
    Point p1 = Point(CVector3(-3, -2, 0),Type::via, "Start");
    Point p2 = Point(CVector3(9, -5, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,4.12);
    REQUIRE(round(result.GetX()) == 1 );// around 3.5
    REQUIRE(round(result.GetY()) == -3 );// around 0.5
  }
  SECTION( "cross y negative 2" ) {
    Point p1 = Point(CVector3(1, -6, 0),Type::via, "Start");
    Point p2 = Point(CVector3(-5, -3, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,2.24);
    REQUIRE(round(result.GetX()) == -1 );// around 3.5
    REQUIRE(round(result.GetY()) == -5 );// around 0.5
  }
}

TEST_CASE("diagonal crossing y and x", "[predicted location]"){
  Point p = Point(1,2,0,Type::station,std::string("FB21.Initial"));
  CFootBotEntity* fb =new CFootBotEntity("FB21","fdc",p.getCVector());
  Robot r = Robot(fb, &p);
  SECTION( "cross y and x posivite" ) {
    Point p1 = Point(CVector3(4, -2, 0),Type::via, "Start");
    Point p2 = Point(CVector3(-1, 3, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2,1.14);
    //REQUIRE((x < 0.1 && x > -0.1));
    REQUIRE(round(result.GetX()) == 3 );// around 3.5
    REQUIRE(round(result.GetY()) == -1 );// around 0.5
  }
    SECTION( "cross x and y negative" ) {
    Point p1 = Point(CVector3(2, -3, 0),Type::via, "Start");
    Point p2 = Point(CVector3(-2, 1, 0),Type::via, "end");
    CVector3 result = r.getPosition(p1,p2, 1.14);
    REQUIRE(round(result.GetX()) == 1 );// around 3.5
    REQUIRE(round(result.GetY()) == -2 );// around 0.5
  }
}
