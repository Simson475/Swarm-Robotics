#include "Path.hpp"

std::vector<int> Path::asWaypointPlan(){
  std::vector<int> waypointPlan = {1};//First element will be "popped" before being used
  for(Action a : actions){
    waypointPlan.push_back(a.endVertex->getId());
  }
  return waypointPlan;
}

std::string Path::toString(){
  std::string str = "";
  str += "v" + std::to_string(actions[0].startVertex->getId()) + "(" + std::to_string(actions[0].timestamp) + ") ";
  for(Action a : actions){
    str += "v" + std::to_string(a.endVertex->getId()) + "(" + std::to_string(a.timestamp + a.duration) + ") ";
  }
  return str;
}

Path Path::operator+(const Path& path){
  auto actions = this->actions;
  for (auto a : path.actions){
    actions.push_back(a);
  }
  Path p = {actions, this->cost + path.cost};
  return p;
}

void Path::operator=(const Path &other){
  this->actions = other.actions;
  this->cost = other.cost;
}

void Path::operator=(std::vector<Action> actions){
  this->actions = actions;
  this->cost = actions.size() > 0 ? actions.front().timestamp : 0;
  for(auto& a : actions){
    cost += a.duration;
  }
}