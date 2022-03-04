#include "Agent.hpp"
#include "point.hpp"
#include <queue>

void Agent::createPath(std::vector<Point> path){
    std::vector<Action> actions{};
    for (int i=0 ; i < path.size()-1; i++){
        Action temp{};
        temp.startVertex = path[i];
        temp.endVertex = path[i+1];
        actions.push_back(temp);
        //TODO
    }
}