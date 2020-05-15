#include "stratego.h"

std::string stratego::getSingleTrace(queryType type, std::string path) {
  std::string uppaalFormulaResults = createModel(type, path);
  //Used for storing the full trace
  Simulation parsed;
  try{
     parsed = parseStr(uppaalFormulaResults, FORMULA_NUMBER, type);
  }
  catch(std::runtime_error &e){
    throw e;
  }
  for(auto& p :parsed.trace){
    std::cout<< p.first << " time: " << p.second.timeStamp << " location: " <<p.second.via->getId()<<std::endl;
  }
  std::string viasPath = "";
  for(auto& p :parsed.points){
      viasPath += "(" + std::to_string(int(p->getX())) +";" + std::to_string(int(p->getY())) + ") ";
  }
  return viasPath;
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
  if (stream) {
    while (!feof(stream)){
      if (fgets(buffer, max_buffer, stream) != NULL){
        result.append(buffer);
      }
    }
  }
  return result;
}

Simulation combineTraces(std::vector<std::vector<Data>> traces, stratego::queryType type){
    std::vector<shared_ptr<Point>> points;
    std::vector<std::pair<std::string, Data>> mergedTraces(traces[0].size() +  traces[1].size());
    if(type == stratego::queryType::waypoints){
        int arrivalId = 1, delayId = 0;
        mergedTraces.at(0) = std::make_pair("Arrives at",traces[0][0]);
        points.push_back(traces[0][0].via);
        for(auto i = 1; i < mergedTraces.size(); i ++){
            if(i % 2 == 1){
                points.push_back(traces[0][arrivalId].via);
                mergedTraces.at(i) = std::make_pair("Arrives at",traces[0][arrivalId++]);
            }
            else{
                mergedTraces.at(i) = std::make_pair("Delays till",traces[1][delayId++]);
            }
        }
    }
    if(type == stratego::queryType::stations){
        int arrivalId = 0, delayId = 1;
        mergedTraces.at(0) = std::make_pair("Arrives at",traces[1][0]);
        points.push_back(traces[0][0].via);
        for(auto i = 1; i < mergedTraces.size(); i ++){
            if(i % 2 == 1){
                mergedTraces.at(i) = std::make_pair("Delays till",traces[1][delayId++]);
            }
            else{
                points.push_back(traces[0][arrivalId].via);
                mergedTraces.at(i) = std::make_pair("Arrives at",traces[0][arrivalId++]);
            }
        }
    }
    return Simulation{points, mergedTraces};
}

Simulation stratego::parseStr(std::string result, int formula_number, queryType type) {
  int run_number = 0;
  const size_t startIndex =
      result.find("Verifying formula " + std::to_string(formula_number));
  const size_t stopIndex = result.find(
      "Verifying formula " +
      std::to_string(formula_number +
                     1)); // Equal to std::string::npos if not found.
  std::string formula;
  if(startIndex == std::string::npos){
    throw std::runtime_error("Failed interecting with Uppaal, check Uppaal path");

  }
  if (stopIndex == std::string::npos) {
    formula = result.substr(startIndex);
  } else {
    formula = result.substr(startIndex, stopIndex - startIndex);
  }

  std::stringstream ss{formula};
  std::vector<Simulation> values;
  std::vector<std::vector<Data>> runs;
  std::string line;
  std::getline(ss, line); // Verifying formula \d+ at <file:line>
  std::getline(ss, line); // -- Formula is (not)? satisfied.

  if (ss.eof()) {
    return combineTraces(runs, type);
  }

  std::getline(ss, line);
  while (line.size() > 0) {
    std::vector<Data> e = parseValue(ss, line, run_number, type);
    if (e.size() != 0) {
      runs.push_back(e);
    }
    if (ss.eof()) {
      break;
    }
  }
  return combineTraces(runs, type);
}
//method to determine if value is worth storing
bool storeValue(int run_number, int id_of_value, bool& even, double time, stratego::queryType type){
  //ensures that only valuable information is stored while checking for stations
  if(type == stratego::queryType::stations){
      if(run_number == 1 && (id_of_value < 3 || id_of_value % 2 == 0)){
        return false;
      }
      if(run_number == 2 && id_of_value == 0){
        return false;
      }
      if(run_number == 2 && id_of_value == 1 && time != 0){
        even = false;
        return false;
      }
      if(run_number == 2 && id_of_value > 2-even && id_of_value % 2 == even){
        return false;
      }
      return true;
  }
  //ensures that only valuable information is stored while checking for waypoints
  else {
    if(run_number == 1 && (id_of_value == 0 || id_of_value % 2 == 0)){
        return false;
    }
    if(run_number == 2 && (id_of_value < 3 || id_of_value % 2 == 0)){
        return false;
    }
    return true;
  }

}
std::vector<Data> stratego::parseValue(std::istream &ss, std::string &line, int &run_number, queryType type) {
  Map_Structure &sMap = Map_Structure::get_instance();
  std::string name = line.substr(0, line.length() - 1);
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
  std::vector<Data> values;
  std::smatch line_match;
  std::smatch pair_match;

  while (std::regex_match(line, line_match, line_pattern)) {
    std::string pairs = line_match[2];
    //if the run_number is equal to zero it means that it contains
    //only the first arrival point, thus it can be ignored
    if(run_number != 0) { 
        int id_of_value = 0;
        //if station_eta == 0 we use uneven numbers from list
        //if we have more then 0 estimated time to station we use even numbers
        bool even = true;
            // parse the list of value pairs
            while (std::regex_search(pairs, pair_match, pair_pattern)) {
                double time = std::stod(pair_match[1]);
                int value = std::stoi(pair_match[2]);
                //determines if one shall store the value or ignore it
                bool store = storeValue(run_number, id_of_value, even, time, type);
                pairs = pair_match.suffix();
                id_of_value++;
                if(store){
                    auto i = std::find_if(sMap.points.begin(), sMap.points.end(),
                        [&](shared_ptr<Point>& i){return i->getId()==value;});
                    if(i == sMap.points.end()){
                        i = std::find_if(sMap.stations.begin(), sMap.stations.end(),
                         [&](shared_ptr<Point>& i){return i->getId()==value;});
                    }
                    values.push_back(Data{time, (*i)});
                }
            }
        if (ss.eof()) {
            break;
        }
    }
    run_number++;
    std::getline(ss, line);
  }
  return values;
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
    throw std::runtime_error("Static analyzes does not contain end_stations or vias or stations");
  }
  return result;
}

std::vector<int> getEveryRobotID() {
  std::vector<int> result;
  
  try {
    std::ifstream i("../config/dynamic_config.json");
    if(i.fail()) throw std::runtime_error("Failed opening /config/dynamic_config.json");
    nlohmann::json j = nlohmann::json::parse(i);
    auto &uuid = j.at("robot_info_map");
    if (uuid.is_object()) {
      int k = 0;
      auto obj = uuid.get<nlohmann::json::object_t>();
      for (auto &id : obj)
        result.push_back(stoi(id.first.c_str()));
    }
    else std::cout << "robot_info_map not found, no other robots?"<<std::endl;
  }
  catch (std::runtime_error &e) {
    throw e;
  }
  return result;
}

void stratego::createWaypointQ() {
  std::vector<int> robots = getEveryRobotID();
  std::vector<int> waypoints = getAllWaypoints();
  std::ofstream query;

  query.open("../config/waypoint_scheduling.q");
  query << "strategy realistic = minE (total) [<="+ STRATEGY_UNDER+"] {\\" << std::endl;
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
  query << "simulate 1 [<= "+ STRATEGY_UNDER+"] {Robot.Holding, Robot.cur_waypoint, "
  "Robot.dest_waypoint} under realistic"
        << std::endl;
  query << "saveStrategy(\"waypoint_strategy.json\", realistic)" << std::endl;
  query.close();
}