#include "Agent.hpp"

Agent::Agent(int id, TestController* controller){
    this->id = id;
    this->bot = controller;
    controller->setAgentId(id);
    auto initVertex = ExperimentData::get_instance().getGraph()->getVertices()[controller->getLastLocation()];
    this->currentAction = Action(0, initVertex, initVertex, 0);
}

void Agent::createPath(std::vector<Point> plan){
    /*if (plan.size() < 2) { return; }// Prevent nasty things
    
    std::vector<Action*> actions{};
    std::vector<std::vector<float>> matrix = Map_Structure::get_instance().getRealShortestDistanceMatrix();
    float robotspeed = 0.1;         //TODO FIND REAL SPEED
    float startTime = 0;
    for (std::vector<Point>::size_type i=0; i < plan.size()-2; i++){
        Action* action = new Action();
        action->startVertex = plan[i];
        action->endVertex = plan[i+1];
        action->timestamp = startTime;
        float distance = matrix[action->startVertex.getId()][action->endVertex.getId()];
        float cost = distance / robotspeed;
        action->cost = cost;//WARNING cost changed!
        startTime += cost;
        actions.push_back(action);
    }
    this->plan = plan;
    this->path = Path{actions: actions, cost: startTime };*/
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

Location Agent::getLocation(){
    return bot->getCurrentLocation();
}

int Agent::getId(){
    return this->id;
}

Action Agent::getCurrentAction(){
    Error::log("j");
    Error::log(std::to_string(currentAction.startVertex->getId()));
    return currentAction;
}

int Agent::getTimeAtVertex(std::shared_ptr<Vertex> vertex){
    return (vertex->getId() == this->bot->getStationPlan().front()) ? 10 : 1;//TODO actual times!
}

std::shared_ptr<Vertex> Agent::getGoal(){
    if (bot->getStationPlan().empty()){
        Error::log("Warning: An agents goal was requested, but had none.");
        return getCurrentAction().endVertex;
    }
    // Return the vertex for the front of station plan
    return ExperimentData::get_instance().getGraph()->getVertices()[bot->getStationPlan().front()];
}