#include "stratego.h"
#include "map_structure.h"

bool contains(int id, std::vector<shared_ptr<Point>> points) {
  for (auto &point : points) {
    if (point->getId() == id)
      return true;
  }
  return false;
}

std::string extractCoordinates(std::vector<SimulationExpression> &stationPath,
                                 std::vector<shared_ptr<Point>>& vias, int run) {
  std::vector<shared_ptr<Point>> remaining;
  std::string points = "";
  for (auto &value: stationPath[run].runs[0].values) {
    for (auto &via: vias) {
      if (via->getId() == value.value) {
        if ((value.time == 0 && value.value == 0) || value.value == -1) {
        } else if (!contains(via->getId(), remaining)) {
          remaining.push_back(via);
          points += "(" + std::to_string(int(via->getX())) +
                    ";" + std::to_string(int(via->getY())) + ") ";
        }
      }
    }
  }
  return points;
}

std::string stratego::getSingleTrace(queryType type, std::string path) {
  std::string uppaalFormulaResults = createModel(type, path);

  //Stores each trace of result in std::vector<SimulationExpression> parsed
  //such as:: Robot.initial_station
  //Robot.converted_cur()
  //Robot.converted_dest()
  std::vector<SimulationExpression> parsed;
  try{
     parsed = parseStr(uppaalFormulaResults, ACTION);
  }
  catch(const char* msg){
    throw msg;
  }
  Map_Structure &sMap = Map_Structure::get_instance();
  if (type == queryType::stations)
    // Run 2 is used for stations to extract data
    return extractCoordinates(parsed, sMap.stations, 2);
  else
    // Run 1 is used for waypoints to extract data
    return extractCoordinates(parsed, sMap.points, 1);
}

std::string stratego::createModel(queryType type, std::string path) {
  std::string terminalCommand = LIBRARY_PATH;
  if (type == queryType::stations) {
    terminalCommand += " && cd ../config/ && " + path + VERIFYTA + " " +
                       STATION_MODEL_PATH + " " + STATION_QUERY_PATH;
  } else {
    terminalCommand += " && cd ../config/ && " + path + VERIFYTA + " " +
                       WAYPOINT_MODEL_PATH + " " + WAYPOINT_QUERY_PATH;
    createWaypointQ();
  }
  
  std::string result;
  FILE *stream;
  const int max_buffer = 256;
  char buffer[max_buffer];
  stream = popen(terminalCommand.c_str(), "r");
  std::cout << terminalCommand << std::endl;
  if (stream) {
    while (!feof(stream)){
      if (fgets(buffer, max_buffer, stream) != NULL){
        result.append(buffer);
      }
    }
  }
  return result;
}

std::vector<SimulationExpression> stratego::parseStr(std::string result,
                                                     int formula_number) {
  const size_t startIndex =
      result.find("Verifying formula " + std::to_string(formula_number));
  const size_t stopIndex = result.find(
      "Verifying formula " +
      std::to_string(formula_number +
                     1)); // Equal to std::string::npos if not found.
  std::string formula;
  if(startIndex == std::string::npos){
    
    throw "Failed interecting with Uppaal, check Uppaal path";

  }
  if (stopIndex == std::string::npos) {
    formula = result.substr(startIndex);
  } else {
    formula = result.substr(startIndex, stopIndex - startIndex);
  }

  std::stringstream ss{formula};
  std::vector<SimulationExpression> values;

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
class SimulationParseException : public std::exception {
  std::string message;

public:
  SimulationParseException(const std::string &inmessage) : message(inmessage) {}

  const char *what() const noexcept override { return message.c_str(); }
};

SimulationExpression stratego::parseValue(std::istream &ss, std::string &line) {
  std::string name = line.substr(0, line.length() - 1);
  std::vector<SimulationTrace> runs;

  // Read run
  std::getline(ss, line);

  std::string time;
  std::string value;

  // Matches full row of simulation results on the form: [run_number]:
  // (time,value) (time,value)... group 1 == run_number, group 2 == all
  // time-value pairs.
  static const std::regex line_pattern{
      R"(\[(\d+)\]:((?: \(-?\d+(?:\.\d+)?,-?\d+\))*))"};
  // matches single (time,value) pair.
  // group 1 == time, group 2 == value.
  static const std::regex pair_pattern{R"( \((-?\d+(?:\.\d+)?),(-?\d+)\))"};

  std::smatch line_match;
  std::smatch pair_match;
  while (std::regex_match(line, line_match, line_pattern)) {
    std::vector<TimeValuePair> values;
    int run_number = std::stoi(line_match[1]);
    std::string pairs = line_match[2];

    // parse the list of value pairs
    while (std::regex_search(pairs, pair_match, pair_pattern)) {
      double time = std::stod(pair_match[1]);
      int value = std::stoi(pair_match[2]);
      values.push_back(TimeValuePair{time, value});
      pairs = pair_match.suffix();
    }

    runs.push_back(SimulationTrace{run_number, values});
    if (ss.eof()) {
      break;
    }

    std::getline(ss, line);
  }

  return SimulationExpression{name, runs};
}

std::string stratego::GetStdoutFromCommand(std::string cmd) {
  std::string data;
  FILE *stream;
  const int max_buffer = 256;
  char buffer[max_buffer];
  cmd.append(" 2>&1");

  stream = popen(cmd.c_str(), "r");
  if (stream) {
    while (!feof(stream))
      if (fgets(buffer, max_buffer, stream) != NULL)
        data.append(buffer);
    pclose(stream);
  }
  return data;
}

void fetchData(std::vector<int> &result, std::string from, nlohmann::json &j) {
  auto &array = j.at(from);
  for (auto &&val : array) {
    result.push_back(stoi(val.dump().c_str()));
  }
}

std::vector<int> getAllWaypoints() {
  std::vector<int> result;
  std::ifstream i("../config/static_config.json");
  nlohmann::json j = nlohmann::json::parse(i);
  try {
    fetchData(result, "end_stations", j);
    fetchData(result, "stations", j);
    fetchData(result, "vias", j);
  }
  catch(...){
    throw "Static analyzes does not contain end_stations or vias or stations";
  }
  return result;
}

std::vector<int> getEveryRobotID() {
  std::vector<int> result;
  std::ifstream i("../config/dynamic_config.json");
  nlohmann::json j = nlohmann::json::parse(i);
  try {
    auto &uuid = j.at("robot_info_map");
    if (uuid.is_object()) {
      int k = 0;
      auto obj = uuid.get<nlohmann::json::object_t>();
      for (auto &id : obj)
        result.push_back(stoi(id.first.c_str()));
    }
  }
  catch (std::exception e) {
    //@todo: Add proper error handling
  }
  return result;
}

void stratego::createWaypointQ() {
  std::vector<int> robots = getEveryRobotID();
  std::vector<int> waypoints = getAllWaypoints();
  std::ofstream query;

  query.open("../config/waypoint_scheduling.q");
  query << "strategy realistic = minE (total) [<=500] {\\" << std::endl;
  query << "Robot.location,Robot.dest,Robot.cur_waypoint,Robot.dest_waypoint,\\"
        << std::endl;
  int k = 2; // since we always have at least one robot;
  for (auto i = 0; i < robots.size(); i++) {
    query << std::string("OtherRobot(") + std::to_string(k) +
                 std::string(").location,\\")
          << std::endl;
    query << std::string("OtherRobot(") + std::to_string(k) +
                 std::string(").cur,\\")
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
  query << "simulate 1 [<= 500] {Robot.cur_waypoint, Robot.dest_waypoint, "
           "Robot.Holding} under realistic"
        << std::endl;
  query << "saveStrategy(\"waypoint_strategy.json\", realistic)" << std::endl;
  query.close();
}