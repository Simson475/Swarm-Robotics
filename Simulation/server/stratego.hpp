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
#define LIBRARY_PATH std::string("export LD_LIBRARY_PATH=$(pwd)/Library")
#define UPPAAL_PATH std::string("~/Desktop/uppaalStratego/bin-Linux/verifyta.bin")
#define STATION_MODEL_PATH std::string("../../station_scheduling.xml")
#define STATION_QUERY_PATH std::string("../../station_scheduling.q")
#define WAYPOINT_MODEL_PATH std::string("../../waypoint_scheduling.xml")
#define WAYPOINT_QUERY_PATH std::string("waypoint_scheduling.q")

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


namespace stratego{
//Used for chosing between two different calls of Uppaal
//Big planning a.k.a. Station planning
//And the small one a.k.a. Waypoint (between stations) planning
enum queryType { stations, waypoints };
//Used for determination which trace shall be read in order to get 
//the list of coordinates which robot needs to travel to
enum traceType {initial_station, converted_cur, converted_dest};

//Depending if it's stations or waypoints call Uppaal and returns
//points which has to be taken in order to get to the destination
std::string getSingleTrace(std::string robotName, queryType type);

//Creates waypoint query file depending on all vias and other robots 
void createWaypointQ(std::string robotName);

//Opens terminal and calls Uppaal with given data
std::string createModel(std::string robotName, queryType type);

//Retreives data from terminal, returned by Uppaal
std::string GetStdoutFromCommand(std::string cmd);

//Helper method forparseStr
Simulation parseValue(std::istream &ss,std::string &line);

//Parses data from GetStdoutFromCommand
std::vector<Simulation> parseStr(std::string result, int formula_number);
};
#endif