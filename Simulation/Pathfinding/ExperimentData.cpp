#include "ExperimentData.hpp"

std::shared_ptr<Graph> ExperimentData::getGraph(){
    if ( graph != nullptr ) { return graph; }

    mapStructureGraph = std::make_shared<MapStructureGraph>(Map_Structure::get_instance());
    graph = std::static_pointer_cast<Graph>(mapStructureGraph);
    return graph;
}

std::vector<std::shared_ptr<Agent>> ExperimentData::getAgents(){
    if ( ! agents.empty()) { return agents; }

    auto &tBotMap = argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    std::vector<std::shared_ptr<Agent>> _agents;
    int id = 0;
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        TestController* controller = dynamic_cast<TestController*>(&(pcBot->GetControllableEntity().GetController()));
        _agents.emplace_back(std::make_shared<Agent>(id, controller));
        id++;
    }

    agents = _agents;

    return agents;
}

bool ExperimentData::requestSolution(int agentId){
    /* If more than the requesting agent does not have a station plan
     * we make the agent wait. */
    for (auto a : getAgents()){
        if (a->getId() != agentId && a->getBot()->getStationPlan().empty() && !a->getBot()->isFinished()){
            return false;
        }
    }
    Error::log("Going to find a solution\n");
    auto agentInfos = getAgentsInfo();

    Solution solution = HighLevelCBS::get_instance()
        .findSolution(getGraph(), agentInfos, LowLevelCBS::get_instance());
    
    // Distribute paths
    distributeSolution(solution);

    return true;
}

void ExperimentData::distributeSolution(Solution solution){
    for (auto agent : getAgents()){
        auto path = solution.paths[agent->getId()];
        // Set path
        agent->getBot()->setPath(path);
    }
}

std::vector<AgentInfo> ExperimentData::getAgentsInfo(){
    auto agents = getAgents();
    int size = agents.size();
    std::vector<AgentInfo> agentInfo{(long unsigned int)size};
    for (int i = 0; i < size; ++i){
        agentInfo[i] = agents[i]->getAgentInfo();
    }
    return agentInfo;
}

int ExperimentData::getNextStation(int agentId){
    auto stations = getStations();
    auto currentStations = getLastAgentGoals();
    int currentStationCount = currentStations.size();
    int station;
    bool stationIsFree = false;
    do {
        stationIsFree = true;
        station = stations[nextStation];
        nextStation = ++nextStation == (int)stations.size() ? 0 : nextStation;
        for (int i = 0; i < currentStationCount; ++i){
            if (currentStations[i] == station && agentId != i){
                stationIsFree = false;
                break;
            }
        }
    }
    while(stationIsFree == false);
    return station;
}

std::vector<int> ExperimentData::getStations(){
    if (mapStructureGraph == nullptr) {
        getGraph();
    }
    return mapStructureGraph->getStations();
}

std::vector<int> ExperimentData::getLastAgentGoals(){
    return this->lastAgentGoals;
}

void ExperimentData::setLastAgentGoal(int agentId, int goalStation){
    this->lastAgentGoals[agentId] = goalStation;
}

float ExperimentData::getSimulationTime(){
    return argos::CSimulator::GetInstance().GetSpace().GetSimulationClock();
}