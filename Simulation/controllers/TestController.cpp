#include "TestController.hpp"
#include <iostream>

std::vector<int> TestController::constructStationPlan(){    
    // std::cout << "Testing station plan";
    // TestControllers dont have an ID before first call of waypoint plan??
    return { ExperimentData::get_instance().getNextStation(this->agentId) };
}

std::vector<int> TestController::constructWaypointPlan(){
    #ifdef DEBUG_LOGS_ON
    Error::log("Agent" + std::to_string(agentId) + " constructs waypoint plan\n");
    #endif
    // If we already have a plan
    if ( ! path.actions.empty()){
        if(resyncNeeded()){
            #ifdef DEBUG_LOGS_ON
            Error::log("Conflict because of desync \n");
            #endif
            ExperimentData::get_instance().requestSolution(agentId);
        }
        return getNextPointAndUpdateState();
    }

    //We do not have a plan
    if (ExperimentData::get_instance().requestSolution(agentId)){
        return getNextPointAndUpdateState();
    }

    //If we requested a solution, but did not get one, we just wait where we are.
    auto currentVertex = ExperimentData::get_instance().getGraph()->getVertices()[lastLocation];
    // This should only happen at the very first time step before all agents have received a station plan
    path.actions.push_back(Action(0, currentVertex, currentVertex, 1));
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
    if (action.isWaitAction()){
        startWaiting();
    }
    #ifdef DEBUG_LOGS_ON
    Error::log("Agent" + std::to_string(agentId) + " going to take action: " + action.toString() + "\n");
    #endif
    return vec;
}

bool TestController::resyncNeeded(){
    //Resync if Argos caused desync
    Action& action = getCurrentAction();
    float actualTime = ExperimentData::get_instance().getSimulationTime();
    float timeDiff = actualTime - action.timestamp;
    #ifdef DEBUG_LOGS_ON
    Error::log("timeDifference is" + std::to_string(timeDiff) + "\n");
    #endif
    action.timestamp += timeDiff;
    for (Action& a : path.actions){
        a.timestamp += timeDiff;
    }

    //See if the resynced times cause conflicts
    // Create a solution from the current paths so we can find conflicts on it
    std::vector<Path> allAgentPaths;
    for (std::shared_ptr<Agent> agent : ExperimentData::get_instance().getAgents()){
        allAgentPaths.push_back(agent->getBot()->path);
        
    }
    std::shared_ptr<ConstraintTree> root = std::make_shared<ConstraintTree>();
    root->setSolution(allAgentPaths);
    int conflicts = root->findConflicts().size();
    
    return conflicts;
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
    waitClock++;
    // If waiting action is over change state to moving so we can get the next action automatically
    if (currentAction.duration <= waitClock){
        currentState = state::moving;
        resetWaypointPlan();
    }
}

void TestController::setCurrentAction(Action action){
    this->currentAction = action;
}

Action& TestController::getCurrentAction(){
    return this->currentAction;
}

void TestController::startWaiting(){
    waitClock = 0;
    currentState = state::waiting;
}

REGISTER_CONTROLLER(TestController, "TestController")