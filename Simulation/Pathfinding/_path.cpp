#include "_path.hpp"

std::vector<int> Path::asWaypointPlan(){
  std::vector<int> waypointPlan = {1};//First element will be "popped" before being used
  for(Action a : actions){
    waypointPlan.push_back(a.endVertex->getId());
  }
  return waypointPlan;
}

std::string Path::toString(){
  std::string str = "";
  str += "v" + std::to_string(actions[0].startVertex->getId());
  for(Action a : actions){
    str += "v" + std::to_string(a.endVertex->getId()) + "t" + std::to_string(a.timestamp + a.duration) + " ";
  }
  return str;
}