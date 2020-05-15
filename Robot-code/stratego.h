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
#include "map_structure.h"

#define FORMULA_NUMBER 2
#define STRATEGY_UNDER std::string("500")
#define LIBRARY_PATH std::string("export LD_LIBRARY_PATH=$(pwd)/external/lib")
#define DEFAULT_UPPAAL_PATH std::string("~/Desktop/uppaalStratego/")
#define VERIFYTA std::string("bin-Linux/verifyta.bin")
#define STATION_MODEL_PATH std::string("station_scheduling.xml")
#define STATION_QUERY_PATH std::string("station_scheduling.q")
#define WAYPOINT_MODEL_PATH std::string("waypoint_scheduling.xml")
#define WAYPOINT_QUERY_PATH std::string("waypoint_scheduling.q")

struct Data{
    double timeStamp;
    std::shared_ptr<Point> via;
};
struct Simulation{
    //extraction of only points from the full trace
    std::vector<std::shared_ptr<Point>> points;
    //full trace, where the string shows the command e.g. Delay in stations till time.  
    //and Data provides with double time stamp and point for the location
    std::vector<std::pair<std::string,Data>> trace;
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
std::string getSingleTrace(queryType type, std::string path);

//Creates waypoint query file depending on all vias and other robots 
void createWaypointQ();

//Opens terminal and calls Uppaal with given data
std::string createModel(queryType type, std::string path);

//Retreives data from terminal, returned by Uppaal
std::string GetStdoutFromCommand(std::string cmd);

//Helper method forparseStr
std::vector<Data> parseValue(std::istream &ss,std::string &line, int &run_number, queryType type);

//Parses data from GetStdoutFromCommand
Simulation parseStr(std::string result, int formula_number, queryType type);
};
#endif