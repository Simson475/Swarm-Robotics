#include "TestController.hpp"
#include <iostream>

std::vector<int> TestController::constructStationPlan(){
    // std::cout << "Testing station plan";
    return SingleThreadBotGreedy::constructStationPlan();
}
std::vector<int> TestController::constructWaypointPlan(){
    // If we already have a plan
    if ( ! path.actions.empty()){
        return getNextPointAndUpdateState();
    }
    //We do not have a plan

    if (ExperimentData::get_instance().requestSolution(agentId)){
        return getNextPointAndUpdateState();
    }
    
    //If we requested a solution, but did not get one, we just wait where we are.
    std::vector<int> vec;
    
    // Create a single point waypoint plan (it is cleared after it is reached anyways)
    vec.push_back(ExperimentData::get_instance().getGraph()->getVertices()[lastLocation]->getId());
    
    // Set state depending on the action
    currentState = state::waiting;

    return vec;
}

std::vector<int> TestController::getNextPointAndUpdateState(){
    std::vector<int> vec;
    Action action = path.actions.front();
    path.actions.erase(path.actions.begin());
    
    // Create a single point waypoint plan (it is cleared after it is reached anyways)
    vec.push_back(action.endVertex->getId());
    
    // Set state depending on the action
    currentState = (action.startVertex->getId() == action.endVertex->getId()) ? state::waiting : state::moving;

    return vec;
}

std::vector<Point> TestController::findOptimalPath(){
    return sMap.findPath(lastLocation, stationPlan.front());
}

int TestController::getAgentId(){
    return this->agentId;
}

void TestController::setAgentId(int id){
    this->agentId = id;
}

Location TestController::getCurrentLocation(){
    return currentLocation;
}

void TestController::reachedPointEvent(int id){
    currentAction = path.actions.front();
    path.actions.erase(path.actions.begin());
    // If the action is a wait action, we set the location to the vertex
    // else we set the location to the edge.
    if (currentAction.startVertex == currentAction.endVertex){
        currentLocation.type = ELocationType::VERTEX_LOCATION;
        currentLocation.vertex = currentAction.startVertex;
    }
    else{
        currentLocation.type = ELocationType::EDGE_LOCATION;
        for (std::shared_ptr<Edge> edge : currentAction.startVertex->getEdges()){
            if (edge->getEndVertex()->getId() == currentAction.endVertex->getId()){
                currentLocation.edge = edge;
                break;
            }
        }
    }
    resetWaypointPlan();//Needed for RobotInterface ControlStep logic
}

REGISTER_CONTROLLER(TestController, "TestController")