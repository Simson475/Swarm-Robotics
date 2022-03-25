#include "_path.hpp"

std::vector<int> Path::asWaypointPlan(){
  std::vector<int> waypointPlan = {};//First element will be "popped" before being used

  if (actions.size()>0) waypointPlan.push_back(actions[0]->startVertex.getId());
  for(Action* a : actions){
    waypointPlan.push_back(a->endVertex.getId());
  } 
    // std::cout << "\n\n\n\n\n";
    // std::cout << "test";
    // for(size_t i=0; i < waypointPlan.size();i++){
    //    std::cout << waypointPlan[i] << " ";
    // }
    // std::cout << "\n\n\n\n\n";
  
  return waypointPlan;
}