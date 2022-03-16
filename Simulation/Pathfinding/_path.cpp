#include "_path.hpp"

std::vector<int> Path::asWaypointPlan(){
  std::vector<int> waypointPlan = {1};//First element will be "popped" before being used
  for(Action* a : actions){
    waypointPlan.push_back(a->endVertex.getId());
  }
  return waypointPlan;
}