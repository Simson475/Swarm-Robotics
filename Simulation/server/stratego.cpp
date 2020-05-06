#include "stratego.hpp"

std::string stratego::getSingleTrace(std::string robotName, queryType type) {
    std::string uppaalFormulaResults = createModel(robotName, type);
  //Stores each trace of result in std::vector<SimulationExpression> parsed
  //such as:: Robot.initial_station
  //Robot.converted_cur()
  //Robot.converted_dest()
  std::vector<Simulation> parsed;
  try{
     parsed = parseStr(uppaalFormulaResults, FORMULA_NUMBER);
  }
  catch(std::runtime_error &e){
    throw std::runtime_error("Failed parsing uppaal results");
  }
    nlohmann::json mainObj;
    std::string singleTrace = "";
    for (int i = 0; i <parsed.size(); i++){
        if (!(parsed[i].name[0] >= 65 && parsed[i].name[0] <= 90) 
        || (parsed[i].name[0] >= 97 && parsed[i].name[0] <= 122)){
            break;
        }
        nlohmann::json jsonObj;
        jsonObj["name"] = parsed[i].name;
        std::vector<nlohmann::json> subObjs;
        for (int j = 0; j <parsed[i].traces.size(); j++){
            nlohmann::json subObj;
            subObj["number"] = parsed[i].traces[j].berth;
            std::vector<nlohmann::json> subSubObjs;
            for (int k= 0; k <parsed[i].traces[j].values.size(); k++){
                nlohmann::json subSubObj;
                subSubObj["time"] = parsed[i].traces[j].values[k].first;
                subSubObj["value"] = parsed[i].traces[j].values[k].second;
                subSubObjs.push_back(subSubObj);
            }
            subObj["values"] = subSubObjs;
            subObjs.push_back(subObj);
        }
        jsonObj["run"] = subObjs;
        mainObj.push_back(jsonObj);
    }
    return mainObj.dump();
}

std::string stratego::createModel(std::string robotName, queryType type){
    std::string terminalCommand = LIBRARY_PATH;
    if(type == queryType::stations){
        terminalCommand +=" && cd " +robotName+ "/stations/ && "  + " " + UPPAAL_PATH + " " + STATION_MODEL_PATH + " " + STATION_QUERY_PATH;
    }else {terminalCommand +=" && cd " +robotName +"/waypoints/ && " + " " + UPPAAL_PATH + " " + WAYPOINT_MODEL_PATH + " " + WAYPOINT_QUERY_PATH;
    createWaypointQ(robotName);}
    std::string result;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    stream = popen(terminalCommand.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL) result.append(buffer);
                pclose(stream);
    }
    return result;
}


std::vector<Simulation> stratego::parseStr(std::string result, int formula_number){
    const size_t startIndex = result.find("Verifying formula " + std::to_string(formula_number));
    const size_t stopIndex =
        result.find("Verifying formula " +
                    std::to_string(formula_number + 1)); // Equal to std::string::npos if not found.

    std::string formula;
    if (stopIndex == std::string::npos) {
        formula = result.substr(startIndex);
    }
    else {
        formula = result.substr(startIndex, stopIndex - startIndex);
    }

    std::stringstream ss{formula};
    std::vector<Simulation> values;

    std::string line;
    std::getline(ss, line); // Verifying formula \d+ at <file:line>
    std::getline(ss, line); // -- Formula is (not)? satisfied.

    if (ss.eof()) {
        return values;
    }

    std::getline(ss, line);
    while (line.size() > 0) {
        values.push_back(parseValue(ss, line));
        if (ss.eof()) {
            break;
        }
    }

    return values;
}
Simulation stratego::parseValue(std::istream &ss,std::string &line)
{
    std::string name = line.substr(0, line.length() - 1);
    std::vector<SimulationTrace> runs;

    // Read run
    std::getline(ss, line);

    std::string time;
    std::string value;

    // Matches full row of simulation results on the form: [run_number]: (time,value)
    // (time,value)... group 1 == run_number, group 2 == all time-value pairs.
    static const std::regex line_pattern{R"(\[(\d+)\]:((?: \(-?\d+(?:\.\d+)?,-?\d+\))*))"};
    // matches single (time,value) pair.
    // group 1 == time, group 2 == value.
    static const std::regex pair_pattern{R"( \((-?\d+(?:\.\d+)?),(-?\d+)\))"};

    std::smatch line_match;
    std::smatch pair_match;
    while (std::regex_match(line, line_match, line_pattern)) {
        std::vector<std::pair<double,int>> values;
        int run_number = std::stoi(line_match[1]);
        std::string pairs = line_match[2];

        // parse the list of value pairs
        while (std::regex_search(pairs, pair_match, pair_pattern)) {
            double time = std::stod(pair_match[1]);
            int value = std::stoi(pair_match[2]);
            values.push_back(std::make_pair(time, value));
            pairs = pair_match.suffix();
        }

        runs.push_back(SimulationTrace{run_number, values});
        if (ss.eof()) {
            break;
        }

        std::getline(ss, line);
    }

    return Simulation{name, runs};
}



std::string stratego::GetStdoutFromCommand(std::string cmd) {
    std::string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream) {
    while (!feof(stream))
    if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
    pclose(stream);
    }
    return data;
}



void fetchData(std::vector<int>& result, std::string from, nlohmann::json& j){
    auto& array = j.at(from);
        for (auto&& val: array) {
            result.push_back(atoi(val.dump().c_str()));
        }
}
std::vector<int> getAllWaypoints(std::string robotName){
    std::vector<int> result;
  std::ifstream i(robotName+"/waypoints/static_config.json");
  nlohmann::json j = nlohmann::json::parse(i);
  try{
        fetchData(result, "end_stations", j);
        fetchData(result, "stations", j);
        fetchData(result, "vias", j);
  }catch(...){
      throw std::runtime_error("Failed reading end_stations or stations or vias from robot: "+robotName);
  }
  return result;
}

std::vector<int> getEveryRobotID(std::string robotName){
  std::vector<int> result;
  std::ifstream i(robotName+"/waypoints/dynamic_config.json");
  nlohmann::json j = nlohmann::json::parse(i);
  try{
    auto& uuid = j.at("robot_info_map");
    if (uuid.is_object())
    {
        int k = 0;
        auto obj = uuid.get<nlohmann::json::object_t>();
        for (auto& id : obj)
            result.push_back(atoi(id.first.c_str()));
    }
  }catch(std::exception const& e){
      throw std::runtime_error("Failed robot_info_map of robot: " + robotName);
  }
  return result;
}

void stratego::createWaypointQ(std::string robotName) {
    std::vector<int> robots = getEveryRobotID(robotName);
    std::vector<int> waypoints = getAllWaypoints(robotName);
  std::ofstream query;

  query.open(robotName + "/waypoints/waypoint_scheduling.q");
  query << "strategy realistic = minE (total) [<="+STRATEGY_UNDER+"] {\\" << std::endl;
  query << "Robot.location,Robot.dest,Robot.cur_waypoint,Robot.dest_waypoint,\\"
        << std::endl;
  int k = 2; // since we always have at least one robot;
  for (auto i = 0; i < robots.size(); i++) {
      query << std::string("OtherRobot(") + std::to_string(k) +
                   std::string(").location,\\")
            << std::endl;
      query << std::string("OtherRobot(") + std::to_string(k) + std::string(").cur,\\")
            << std::endl;
      query << std::string("OtherRobot(") + std::to_string(k) +
                   std::string(").next.type,\\")
            << std::endl;
      query << std::string("OtherRobot(") + std::to_string(k) +
                   std::string(").next.value,\\")
            << std::endl;
      k++;
  }
  for (auto i = 0; i < waypoints.size(); i++) {
    query << std::string("Robot") + std::string(".visited[") +
                 std::to_string(waypoints[i]) + "],\\"
          << std::endl;
  }
  query << "Robot.dest, Robot.cur_waypoint, Robot.dest_waypoint } -> {\\"
        << std::endl;
  for (auto i = 0; i < waypoints.size(); i++) {
    query << std::string("Waypoint(") + std::to_string(waypoints[i]) +
                 std::string(").num_in_queue,\\")
          << std::endl;
  }
  query << "	Robot.x} : <> Robot.Done" << std::endl;
  query << "simulate 1 [<= "+STRATEGY_UNDER+"] {Robot.cur_waypoint, Robot.dest_waypoint, "
           "Robot.Holding} under realistic"
        << std::endl;
  query << "saveStrategy(\"waypoint_strategy.json\", realistic)" << std::endl;
  query.close();
}