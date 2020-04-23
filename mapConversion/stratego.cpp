#include "stratego.h"
#include "map_structure.h"

bool contains(int id, std::vector<weak_ptr<Point>> points) {
  for (auto &point : points) {
    if (point.lock()->getId() == id)
      return true;
  }
  return false;
}

std::string setRemainingStations(std::vector<SimulationExpression> &stationPath,
                                 std::vector<weak_ptr<Point>> vias, int run) {
  std::vector<weak_ptr<Point>> remaining;
  std::string points = "";
  for (auto i = 0; i < stationPath[run].runs[0].values.size(); i++) {
    for (auto j = 0; j < vias.size(); j++) {
      if (vias[j].lock()->getId() == stationPath[run].runs[0].values[i].value) {
        if (stationPath[run].runs[0].values[i].time == 0 &&
                stationPath[run].runs[0].values[i].value == 0 ||
            stationPath[run].runs[0].values[i].value == -1) {
        } else if (!contains(vias[j].lock()->getId(), remaining)) {
          remaining.push_back(vias[j]);
          points = points + "(" + std::to_string(int(vias[j].lock()->getX())) +
                   ";" + std::to_string(int(vias[j].lock()->getY())) + ") ";
        }
      }
    }
  }
  return points;
}

// What does the boolean value `stations` mean intuitively?
std::string stratego::getSingleTrace(bool stations) {
  // The name `result` is a very generic one.
  // Is the result a map? A return string after simulation?
  // Unless I go to check the code itself for `createModel(..)`, I have no idea of its meaning.
  std::string result = createModel(stations);

  // Is SimulationExpression a SimulationResult?
  std::vector<SimulationExpression> parsed = parseStr(result, ACTION);
  nlohmann::json mainObj;
  std::string singleTrace = "";

  for (int i = 0; i < parsed.size(); i++) {
    // Checks for XX condition as we need to exit on that.
    if (!(parsed[i].name[0] >= 65 && parsed[i].name[0] <= 90) ||
        (parsed[i].name[0] >= 97 && parsed[i].name[0] <= 122)) {
      break;
    }

    // We have several JSON object in the code, but what does the
    // JSON object represent? The names jsonObj and subObj does not
    // contain useful information in that regard. It tells me the
    // data type of some implementation, but it does not help in an
    // intuitive understanding.
    nlohmann::json jsonObj;
    jsonObj["name"] = parsed[i].name;
    std::vector<nlohmann::json> subObjs;

    for (int j = 0; j < parsed[i].runs.size(); j++) {
      nlohmann::json subObj;
      subObj["number"] = parsed[i].runs[j].number;
      std::vector<nlohmann::json> subSubObjs;

      for (int k = 0; k < parsed[i].runs[j].values.size(); k++) {
        nlohmann::json subSubObj;
        subSubObj["time"] = parsed[i].runs[j].values[k].time;
        subSubObj["value"] = parsed[i].runs[j].values[k].value;
        subSubObjs.push_back(subSubObj);
      }

      subObj["values"] = subSubObjs;
      subObjs.push_back(subObj);
    }

    jsonObj["run"] = subObjs;
    mainObj.push_back(jsonObj);
  }

  // The code below might benefit by being put into a helper function
  // that has a good descriptive name.
  Map_Structure &sMap = Map_Structure::get_instance();

  if (stations) {
    // Why use weak_ptr<>? Especially when to do not check the result of `lock()`?
    std::vector<weak_ptr<Point>> container;

    for (auto & : sMap.stations)
      container.push_back(p);
    return setRemainingStations(parsed, container, 2);
  } else
    return setRemainingStations(parsed, sMap.points, 1);
}

std::string stratego::createModel(bool stations) {
  std::string terminalCommand = LIBRARY_PATH;
  if (stations) {
    terminalCommand += " && cd ../config/ && " + UPPAAL_PATH + " " +
                       STATION_MODEL_PATH + " " + STATION_QUERY_PATH;
  } else {
    terminalCommand += " && cd ../config/ && " + UPPAAL_PATH + " " +
                       WAYPOINT_MODEL_PATH + " " + WAYPOINT_QUERY_PATH;
    createWaypointQ();
  }
  std::string result;
  FILE *stream;
  const int max_buffer = 256;
  char buffer[max_buffer];
  stream = popen(terminalCommand.c_str(), "r");
  if (stream) {
    while (!feof(stream))
      if (fgets(buffer, max_buffer, stream) != NULL)
        result.append(buffer);
    pclose(stream);
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
    result.push_back(atoi(val.dump().c_str()));
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
  } catch (std::exception e) {
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
        result.push_back(atoi(id.first.c_str()));
    }
  } catch (std::exception e) {
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