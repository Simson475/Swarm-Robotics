#ifndef STRATEGO
#define STRATEGO
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include "nlohmann/json.hpp"
#include <algorithm>
#include <iostream>
#include <regex>
#include <sstream>

#define FORMULA_NUMBER 2
#define STRATEGY_UNDER std::string("500")
#define LIBRARY_PATH std::string("export LD_LIBRARY_PATH=$(pwd)/../Library")
#define DEFAULT_UPPAAL_PATH std::string("~/Desktop/uppaalStratego/")
#define VERIFYTA std::string("bin-Linux/verifyta")
#define STATION_MODEL_PATH std::string("station_scheduling.xml")
#define STATION_QUERY_PATH std::string("station_scheduling.q")
#define WAYPOINT_MODEL_PATH std::string("waypoint_scheduling.xml")
#define WAYPOINT_QUERY_PATH std::string("waypoint_scheduling.q")

//Used for chosing between two different calls of Uppaal
//Big planning a.k.a. Station planning
//And the small one a.k.a. Waypoint (between stations) planning
enum queryType { stations, waypoints };
//Used for determination which trace shall be read in order to get 
//the list of coordinates which robot needs to travel to
enum traceType {initial_station, converted_cur, converted_dest};

// there can be three different kind of traces, one may refer to enum traceType
struct SimulationTrace {
    int berth;
    //first value is the time when the robot is going to be at given location
    //second value shows which point(x,y,z) it is
    std::vector<std::pair<double, int>> values;
};
//Structure to store each simulation trace with it's name
struct Simulation {
    std::string name;
    std::vector<SimulationTrace> traces;
};

class stratego{
public:
//Depending if it's stations or waypoints call Uppaal and returns
//points which has to be taken in order to get to the destination
static std::string getSingleTrace(queryType type, std::string path);

//Creates waypoint query file depending on all vias and other robots 
static void createWaypointQ();

//Opens terminal and calls Uppaal with given data
static std::string createModel(queryType type, std::string path);

//Retreives data from terminal, returned by Uppaal
static std::string GetStdoutFromCommand(std::string cmd);

//Helper method forparseStr
static Simulation parseValue(std::istream &ss,std::string &line);

//Parses data from GetStdoutFromCommand
static std::vector<Simulation> parseStr(std::string result, int formula_number);

private:
stratego() {}

};
#endif