#include "_path.hpp"

std::vector<int> Path::asWaypointPlan(){
  std::vector<int> waypointPlan = {};//First element will be "popped" before being used

  if (actions.size()>0) waypointPlan.push_back(actions[0]->startVertex.getId());
  for(Action* a : actions){
    waypointPlan.push_back(a->endVertex.getId());
  }
  return waypointPlan;
}