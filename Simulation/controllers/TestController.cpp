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
    auto currentVertex = ExperimentData::get_instance().getGraph()->getVertices()[lastLocation];
    path.actions.push_back(Action(time, currentVertex, currentVertex, 1));
    return getNextPointAndUpdateState();
}

/* PRE: path.actions.size() > 0 */
std::vector<int> TestController::getNextPointAndUpdateState(){
    std::vector<int> vec;
    Action action = path.actions.front();
    path.actions.erase(path.actions.begin());

    /** Update current action and location to the new action
      * If the action is a wait action, we set the location to the vertex
      * else we set the location to the edge. */
    this->setCurrentAction(action);
    updateCurrentLocation(action);
    
    // Create a single point waypoint plan (it is cleared after it is reached anyways)
    vec.push_back(action.endVertex->getId());
    
    // Set state depending on the action
    currentState = (action.startVertex->getId() == action.endVertex->getId()) ? state::waiting : state::moving;

    return vec;
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

void TestController::setPath(Path path){
    this->path = path;
}

/**
 * Called when the robot reached point with the id.
 * 
 * @param id - id of the point/vertex reached
 */
void TestController::reachedPointEvent(int id){
    resetWaypointPlan();//Needed for RobotInterface ControlStep logic
}

/**
 * Update the current location to be the location the robot will be 
 * during the action.
 */
void TestController::updateCurrentLocation(Action action){
    if (action.startVertex == action.endVertex){
        currentLocation.type = ELocationType::VERTEX_LOCATION;
        currentLocation.vertex = action.startVertex;
    }
    else{
        currentLocation.type = ELocationType::EDGE_LOCATION;
        for (std::shared_ptr<Edge> edge : action.startVertex->getEdges()){
            if (edge->getEndVertex()->getId() == action.endVertex->getId()){
                currentLocation.edge = edge;
                break;
            }
        }
    }
}

/* Called when current state == waiting */
void TestController::wait(){
    // If waiting action is over change state to moving so we can get the next action automatically
    if ((currentAction.timestamp + currentAction.duration) <= time){
        currentState = state::moving;
        resetWaypointPlan();
    }
}

void TestController::setCurrentAction(Action action){
    this->currentAction = action;
}

Action TestController::getCurrentAction(){
    return this->currentAction;
}

REGISTER_CONTROLLER(TestController, "TestController")