#ifndef STRATEGO
#define STRATEGO
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <json.hpp>
#include <algorithm>
#include <iostream>
#include <regex>
#include <sstream>

#define ACTION 2
#define LIBRARY_PATH std::string("export LD_LIBRARY_PATH=$(pwd)/../Library")
#define DEFAULT_UPPAAL_PATH std::string("~/Desktop/uppaalStratego/")
#define VERIFYTA std::string("bin-Linux/verifyta.bin")
#define STATION_MODEL_PATH std::string("station_scheduling.xml")
#define STATION_QUERY_PATH std::string("station_scheduling.q")
#define WAYPOINT_MODEL_PATH std::string("waypoint_scheduling.xml")
#define WAYPOINT_QUERY_PATH std::string("waypoint_scheduling.q")


enum queryType { stations, waypoints };

struct TimeValuePair {
    double time;
    int value;
};

struct SimulationTrace {
    int number;
    std::vector<TimeValuePair> values;
};
struct SimulationExpression {
    std::string name;
    std::vector<SimulationTrace> runs;
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
static SimulationExpression parseValue(std::istream &ss,std::string &line);
//Parses data from GetStdoutFromCommand
static std::vector<SimulationExpression> parseStr(std::string result, int formula_number);
private:
stratego() {}

};
#endif