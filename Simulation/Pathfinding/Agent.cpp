#include "SingleThreadBotCBS.hpp"
#include "Path.hpp"
#include "Agent.hpp"
#include "point.hpp"
#include <queue>

void Agent::createPath(std::vector<Point> plan){
    std::vector<Action> actions{};
    std::vector<std::vector<float>> matrix = Map_Structure::get_instance().getRealShortestDistanceMatrix();
    int robotspeed = 100;         //TODO FIND REAL SPEED
    int startTime = 0;
    for (int i=0 ; i < plan.size()-1; i++){
        Action temp{};
        temp.startVertex = plan[i];
        temp.endVertex = plan[i+1];
        temp.timestamp = startTime;
        float distance = matrix[temp.startVertex.getId()][temp.endVertex.getId()];
        int cost = distance / robotspeed;
        temp.cost = cost;
        startTime += cost;
        actions.push_back(temp);
    }
    this->plan = plan;
    this->path = Path{actions: actions, cost: startTime };   
}

void Agent::setBot(SingleThreadBotCBS* bot){
    this->bot = bot;
}
SingleThreadBotCBS* Agent::getBot(){
    return this->bot;
}