#include "Agent.hpp"

void Agent::createPath(std::vector<Point> plan){
    if (plan.size() < 2) { return; }// Prevent nasty things
    
    std::vector<Action*> actions{};
    std::vector<std::vector<float>> matrix = Map_Structure::get_instance().getRealShortestDistanceMatrix();
    float robotspeed = 0.1;         //TODO FIND REAL SPEED
    float startTime = 0;
    for (std::vector<Point>::size_type i=0; i < plan.size()-1; i++){
        Action* action = new Action();
        action->startVertex = plan[i];
        action->endVertex = plan[i+1];
        action->timestamp = startTime;
        float distance = matrix[action->startVertex.getId()][action->endVertex.getId()];
        float cost = distance / robotspeed;
        action->cost = cost;
        startTime += cost;
        actions.push_back(action);
    }
    this->plan = plan;
    this->path = Path{actions: actions, cost: startTime };
}

void Agent::setBot(TestController* bot){
    this->bot = bot;
}
TestController* Agent::getBot(){
    return this->bot;
}

Path Agent::getPath(){
    return path;
}