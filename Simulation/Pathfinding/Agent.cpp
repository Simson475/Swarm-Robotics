#include "Agent.hpp"

Agent::Agent(int id, TestController* controller){
    this->id = id;
    this->bot = controller;
    controller->setAgentId(id);
    auto initVertex = ExperimentData::get_instance().getGraph()->getVertices()[controller->getLastLocation()];
    this->getBot()->setCurrentAction(Action(0, initVertex, initVertex, 0));
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
    return this->getBot()->getCurrentAction();
}

int Agent::getTimeAtVertex(std::shared_ptr<Vertex> vertex){
    return (vertex->getId() == this->bot->getStationPlan().front()) ? 10 : 1;//TODO actual times!
}

std::shared_ptr<Vertex> Agent::getGoal(){
    if (bot->getStationPlan().empty()){
        if(!this->getBot()-> isFinished()) Error::log("Warning: An agents goal was requested, but had none.");
        return getCurrentAction().endVertex;
    }
    // Return the vertex for the front of station plan
    return ExperimentData::get_instance().getGraph()->getVertices()[bot->getStationPlan().front()];
}