#include "robot.hpp"
#include "map_structure.hpp"

Robot::Robot(CFootBotEntity *footBot, Point *initialLoc) {
  initialLocation = initialLoc;
  this->footBot = footBot;
  status = Status::available;
  etaNextStation = 0.0;
  currentPositionId = initialLoc;
  
}
Robot &Robot::operator=(const Robot &that) {
  footBot = that.footBot;
  initialLocation = that.initialLocation;
  status = Status::available;
  etaNextStation = 0.0;
  currentPositionId = initialLocation;
  return *this;
}
Point& Robot::getInitialLoc() {
  return *initialLocation;
}

Point Robot::getNextStation() {
  if (!remainingStations.empty()) {
    return remainingStations[0];
  } else
    return getInitialLoc();
}

void Robot::increment(int i) { stopWatch = i; }
void Robot::setEta(double time) { etaNextStation = time; }
void Robot::setJob(vector<Point> &jobs) {
  for (auto i = 0u; i < jobs.size(); i++) {
    job.push_back(jobs[i]);
  }
}
void Robot::setCurrStationTarget(){
  Map_Structure &sMap = Map_Structure::get_instance();
  currTarget = sMap.getPointPointer(remainingStations.front().getId());
  
  
}
double Robot::getEta() { 
  Map_Structure &sMap = Map_Structure::get_instance();
    etaNextStation= 0;
    argos::CVector3 curr = footBot->GetEmbodiedEntity().GetOriginAnchor().Position;
    for(auto i = 0; i < getRemainingWaypoints().size(); i++){
      if(i == 0) etaNextStation = argos::Distance(remainingWaypoints.front(),curr);
      else etaNextStation += (remainingWaypoints[i-1] - remainingWaypoints[i]).Length(); 
    }
    if(getRemainingWaypoints().size() == 1){ // meaning that the next waypoint is already station
      std::vector<Point> path = sMap.findPath(remainingStations.front().getId(),remainingStations[1].getId() );
      ////std::cout<< "Path for <ETA"<< std::endl;
      for(auto i =0; i < path.size(); i++){
        if(i == 0 ) etaNextStation = argos::Distance(remainingStations.front(), path.front());
        else etaNextStation += argos::Distance(remainingStations[i-1],remainingStations[i]);
      }
    }
     ////std::cout <<etaNextStation<<std::endl;
    return etaNextStation = etaNextStation/ VELOCITY *100; }
    argos::CVector3 getPositionHelper(CVector3 start, CVector3 end, argos::Real x, argos::Real y){
argos::Real result_X, result_Y;
  result_X = start.GetX() + x;
  result_Y = start.GetY() + y;
  if(start.GetX() <= result_X && result_X <= end.GetX() &&
    start.GetY() <= result_Y && result_Y <= end.GetY()){
    return argos::CVector3(result_X, result_Y, 0);
  }
  if(start.GetX() >= result_X && result_X >= end.GetX() &&
    start.GetY() >= result_Y && result_Y >= end.GetY()){
    return argos::CVector3(result_X, result_Y, 0);
  }
  if(start.GetX() <= result_X && result_X <= end.GetX() &&
    start.GetY() >= result_Y && result_Y >= end.GetY()){
    return argos::CVector3(result_X, result_Y, 0);
  }
    if(start.GetX() >= result_X && result_X >= end.GetX() &&
    start.GetY() <= result_Y && result_Y <= end.GetY()){
    return argos::CVector3(result_X, result_Y, 0);
  }
    result_X = start.GetX() - x;
  result_Y = start.GetY() - y;
  return argos::CVector3(result_X, result_Y, 0);
    }
argos::CVector3 Robot::getPosition(CVector3 start, CVector3 end, double distance){
  double x, y, teta;
  y = start.GetY() - end.GetY();
  x = start.GetX() - end.GetX();
  if(x != 0) teta = atan(y/x); 
  else return getPositionHelper(start, end, 0, distance);
  if(y != 0) teta = atan(y/x); 
  else return getPositionHelper(start, end, distance, 0);
  return getPositionHelper(start, end, cos(teta) * distance, sin(teta)*distance);
}
bool checkDelayMatch(double timeToDelay, bool once, double temp){
    if (timeToDelay - temp < 0 && once){
      return true;
    }
    return false;
}
timeResult* Robot::getEtaHelper(std::string id, std::vector<Point> waypoints, argos::CVector3 currPosition, double timeToDelay,
                                 double temp, std::vector<Point>& allWaypoints) {
  if(footBot->GetId() == "fb2"&&id == "fb1"){
    //std::cout<< "-------------------------"<< std::endl;
    //std::cout<< "Current position: "<<currPosition << std::endl;
    //std::cout<< "Time to delay: "<<timeToDelay << std::endl;
     //std::cout<< "temp: "<<temp << std::endl;
     for(auto& way:waypoints){
       //std::cout<<"waypoint: "<<way.getName()<<std::endl;

     }
     
  }
  bool once = true;
  struct added{bool added = false;
  bool station;};
  added check;
  argos::CVector3 position;
  int passedStations =0;
  std::vector<Point> passedWaypoints;
    for(auto i = 0; i < waypoints.size(); i++){
    if(i == 0 && temp == 0) temp = argos::Distance(currPosition,waypoints.front());
    else if(i == 0 && temp !=0) temp += argos::Distance(currPosition,waypoints.front());
    else temp += argos::Distance(waypoints[i-1], waypoints[i]);
    if(!checkDelayMatch(timeToDelay,once,temp)){
        if(waypoints[i].getType()==Type::station) {temp+=7.5* VELOCITY /100; // addition of delay + rotation
        check.station= true;
        check.added= true;
}//requires to convert 6 times units to distance;
        else{temp+=2;check.station= false;} // adding 2 time units for the robots rotation
    }
    if(checkDelayMatch(timeToDelay,once,temp)){
      temp = (timeToDelay - temp) * -1;
      once = false;
      passedStations = i;
      if(check.added){position = waypoints.back();
        Point p = waypoints[i];
        allWaypoints.push_back(p);}
      else if(i == 0) position =getPosition(currPosition,waypoints.front(), temp);
      else position = getPosition(waypoints[i-1], waypoints[i], temp);
    }
    check.added = false;
    if(once) {Point p = waypoints[i];
              passedWaypoints.push_back(p);
              allWaypoints.push_back(p);}
  }
  //std::cout<< "-------------------------"<< std::endl;
  return new timeResult{id,temp, passedStations, passedWaypoints, position, once,allWaypoints   };

}
  void Robot::addEndPoint(){
    Map_Structure &sMap = Map_Structure::get_instance();
    remainingStations.push_back(sMap.getPointByID(1));
  }
timeResult* Robot::getEtaNextRobot(Robot r, double timeToDelay) { 
  Map_Structure &sMap = Map_Structure::get_instance();
  timeToDelay = timeToDelay* VELOCITY /100; // back to distance
  double temp = 0;
  if(r.getWatch()!= -1) {temp = (stationDelay- r.getWatch())/10;} // convert to distance
  argos::CVector3 currPosition;
  std::vector<Point> passedWaypoints;
  int passedStations = 0;
  std::vector<Point> allPassedPoints;
  currPosition = r.getfootBot()->GetEmbodiedEntity().GetOriginAnchor().Position;
  allPassedPoints.push_back(Point(currPosition,Type::via,"CurrentPosition"));
  if(timeToDelay < argos::Distance(currPosition,r.getRemainingWaypoints().front())){
    argos::CVector3 position = getPosition(currPosition,r.getRemainingWaypoints().front(),timeToDelay );
    return new timeResult{r.getfootBot()->GetId(),timeToDelay, 0, passedWaypoints, position, false,allPassedPoints   };
  }

    if(temp >= timeToDelay){
    return new timeResult{r.getfootBot()->GetId(),temp, passedStations, passedWaypoints, currPosition, false ,allPassedPoints};
    }
  timeResult* result = getEtaHelper(r.getfootBot()->GetId(),r.getRemainingWaypoints(), currPosition,timeToDelay,temp, allPassedPoints);
  if(r.remainingStations.size() == 1){
    return nullptr;
  }
   //std::cout<<"ONE: "<<std::endl;
  for(auto& p : allPassedPoints){
    //std::cout<<"AllPassed: "<<p.getName()<<std::endl;
  }
     //std::cout<<"ONE END"<<std::endl;
 
  /*for (int i = 0; i < result->waypointsToPass.size(); i++ ){
                if(!result->found && i != result->waypointsToPass.size())
                  allPassedPoints.push_back(result->waypointsToPass[i]);
              }*/
  if(result->found){
    for(auto i = 1; i < r.getRemainingStations().size();i++){
      std::vector<Point> path = sMap.findPath(r.getRemainingStations()[i-1].getId(),r.getRemainingStations()[i].getId());
      result = getEtaHelper(r.getfootBot()->GetId(), path, r.getRemainingStations()[i-1],timeToDelay,result->distance, allPassedPoints);
              /*for (int i = 0; i < result->waypointsToPass.size(); i++ ){
                if(!result->found && i != result->waypointsToPass.size())
                  allPassedPoints.push_back(result->waypointsToPass[i]);
              }*/
                 //std::cout<<"TWO: "<<std::endl;
  for(auto& p : allPassedPoints){
    //std::cout<<"AllPassed: "<<p.getName()<<std::endl;
  }
     //std::cout<<"TWO END"<<std::endl;
     
      if(!result->found) break;
    }
  }
  if(result->found) return nullptr;
  ////std::cout<< "OtherRobot time " <<result->distance <<std::endl; 
  //result->allPassedPoints = allPassedPoints;
  return result;
}


void Robot::addSinglePickUp(Point pickup) { job.push_back(pickup); }
void Robot::removeFirstStation() {
  remainingStations.erase(remainingStations.begin());
}
void Robot::removeFirstWaypoint() {

  currentPositionId = &Map_Structure::get_instance().getPointByID(remainingWaypoints[0].getId());
  remainingWaypoints.erase(remainingWaypoints.begin());
}
void Robot::cleanJob() { job.clear(); }

Point *Robot::getNextWayPoint() {
  if (!remainingWaypoints.empty()) {
    return &remainingWaypoints[0];
  } else
    return nullptr;
}
void Robot::changeStatus(Status stat) { status = stat; }
bool Robot::contains(int id, vector<Point>& points) {
  for (auto& point: points) {
    if (point.getId() == id)
      return true;
  }return false;
}

vector<Point> Robot::setRemainingStations(vector<Point> allPoints) {
  remainingStations.clear();
  for (auto i = 0; i < stationPath[2].runs[0].values.size(); i++) {
    for (auto j = 0; j < allPoints.size(); j++) {
      if (allPoints[j].getId() == stationPath[2].runs[0].values[i].value) {
        if (stationPath[2].runs[0].values[i].time == 0 &&
            stationPath[2].runs[0].values[i].value == 0) {
        } else if (!contains(allPoints[j].getId(), remainingStations)) {
          remainingStations.push_back(allPoints[j]);
        }
      }
    }
  }
  return remainingStations;
}
void Robot::addWaypoints(vector<Point> path) {
  for(auto& p: path){
    remainingWaypoints.push_back(move(p));
  }
  
    
}
void Robot::updateCurrent(Point *target){
   currentPositionId = target;
 }
vector<Point> Robot::setRemainingWaypoints(vector<Point> &allPoints) {
  remainingWaypoints.clear();
  for (auto i = 0; i < waypointPath[1].runs[0].values.size(); i++) {
    for (auto j = 0; j < allPoints.size(); j++) {
      if (allPoints[j].getId() == waypointPath[1].runs[0].values[i].value) {
        if (waypointPath[1].runs[0].values[i].time == 0 &&
                waypointPath[1].runs[0].values[i].value == 0 ||
            waypointPath[1].runs[0].values[i].value == -1) {
        } else if (!contains(allPoints[j].getId(), remainingWaypoints)) {
          remainingWaypoints.push_back(allPoints[j]);
        }
      }
    }
  }
  if (remainingWaypoints.empty())
    remainingWaypoints.push_back(allPoints[0]);

  return remainingWaypoints;
}
void Robot::converJSONStation(string robotId, string choice) {
  string path;
  if (choice == "Stations") {
    stationPath.clear();
    path = "experiment/scene2/" + robotId + "/" + robotId + "Stations.json";
  }
  if (choice == "Waypoints") {
    waypointPath.clear();
    path = "experiment/scene2/" + robotId + "/" + robotId + "Waypoints.json";
  }
  std::ifstream i(path);

  nlohmann::json js = nlohmann::json::parse(i);
  for (auto i = 0; i < js.size(); i++) {
    SimulationExpression se;
    se.name = js[i].value("name", "");
    SimulationTrace smt;
    for (auto j = 0; j < js[i]["run"].size(); j++) {
      smt.number = js[i]["run"][j].value("number", 0);
      for (auto k = 0; k < js[i]["run"][j]["values"].size(); k++) {
        TimeValuePair tvp;
        tvp.time = js[i]["run"][j]["values"][k].value("time", 0.0);
        tvp.value = js[i]["run"][j]["values"][k].value("value", 0);
        smt.values.push_back(tvp);
      }
      se.runs.push_back(smt);
    }
    if (choice == "Stations")
      stationPath.push_back(se);
    if (choice == "Waypoints")
      waypointPath.push_back(se);
  }
}
std::string Robot::createDynamicJson(vector<Robot> &robots, int n, bool stations) {
  nlohmann::json jsonObj;
  if(stations){
    jsonObj["next_station"] = remainingStations[1].getId();
    jsonObj["next_waypoint"] = remainingStations[0].getId();
  }
  else {
    if(remainingStations.size() == 2){
    jsonObj["next_station"] = remainingStations[0].getId();
    jsonObj["next_waypoint"] = remainingWaypoints[0].getId();
    }
    else {
      jsonObj["next_station"] = remainingStations[0].getId();
    jsonObj["next_waypoint"] = remainingWaypoints[0].getId();
    }
    //else jsonObj["next_station"] = 1;
    
  }
  vector<nlohmann::json> visitedWayPoints;
  jsonObj["visited_waypoints"] = visitedWayPoints;
  vector<nlohmann::json> stationsToVisit;
    for (auto i = 1; i < remainingStations.size();i++) {
      stationsToVisit.push_back(remainingStations[i].getId());
    }
  jsonObj["station_eta"] = getEta();
  jsonObj["stations_to_visit"] = stationsToVisit;
  otherRobotsInf.clear();
  for (auto i = 0; i < robots.size(); i++) {
    if (robots[i].getStatus() != Status::available && i != n) {
      vector<Point> p;
      argos::Real dist = argos::Distance(getRemainingWaypoints().front(), 
                          getfootBot()->GetEmbodiedEntity().GetOriginAnchor().Position);
      dist = dist / VELOCITY *100;
        if(stopWatch != -1) dist = dist + ((stationDelay-stopWatch)/10* VELOCITY /100);
      auto otherRobot = getEtaNextRobot(robots[i], dist);
      if(otherRobot != nullptr){
        otherRobotsInf.push_back(otherRobot);
      nlohmann::json outerInfMap;
      nlohmann::json infMap;
      infMap["eta"] = otherRobot->distance/ VELOCITY *100;// convert from distance to time
      infMap["id"] = i;
      nlohmann::json loc;
      loc["x"] = otherRobot->currPosition.GetX();
      loc["y"] = otherRobot->currPosition.GetY();
      infMap["location"] = loc;
      vector<int> ids;
      for (auto j = otherRobot->stationsPassed; j < robots[i].getRemainingStations().size(); j++) {
        ids.push_back(robots[i].getRemainingStations()[j].getId());
      }
      infMap["station_plan"] = ids;
      vector<nlohmann::json> wayPointL;
      for (auto j = 0; j < otherRobot->waypointsToPass.size(); j++) {
        nlohmann::json waypoint;
        waypoint["type"] = "Waypoint";
        waypoint["value"] = otherRobot->waypointsToPass[j].getId();
        wayPointL.push_back(waypoint);
      }

      infMap["waypoint_plan"] = wayPointL;
      jsonObj["robot_info_map"][std::to_string(i)] = infMap;
      }
    }
  }
  std::ofstream out("experiment/scene2/" + footBot->GetId() + "/" +
                    "dynamic_config.json");
  out << std::setw(4) << jsonObj;
  return jsonObj.dump();
}
void Robot::sortJob(vector<vector<float>> shortestDistances)
{
    for(auto i=0;i<job.size()-1;i++)
	  {	
      int k=i;
      float min = INF;	
		  for(auto j=i;j<job.size()-1;j++)
		  {
        float temp;
          if(i == 0){
            temp =shortestDistances[currentPositionId->getId()][job[j].getId()];
          }else temp=shortestDistances[job[i-1].getId()][job[j].getId()];
          
        if(temp<min){
              min = temp;
              k = j;
          }
      }
      std::swap(job[i], job[k]);
    }
    for(auto& j : job)
    remainingStations.push_back(j);

}