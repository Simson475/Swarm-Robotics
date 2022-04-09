#include "ExperimentData.hpp"

std::shared_ptr<Graph> ExperimentData::getGraph(){
    if ( graph != nullptr ) { return graph; }

    auto g = std::make_shared<MapStructureGraph>(Map_Structure::get_instance());
    graph = std::static_pointer_cast<Graph>(g);
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
        // Set the initial vertex in the agents by giving them a wait action at their initial position
        auto currentVertex = getGraph()->getVertices()[controller->getLastLocation()];
        Path path;
        path.actions.push_back(Action(0, currentVertex, currentVertex, 0));
        controller->setPath(path);
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
    // for (auto a1 : agentInfos){
    //     for (auto a2 : agentInfos){
    //         if (a1.getId() != a2.getId() && a1.getGoal() == a2.getGoal()){
    //             Error::log("Impossible to find a solution?\n");
    //             //exit(2);
    //         }
    //     }
    // }

    for (auto a1 : agentInfos){
        Error::log("Agent" + std::to_string(a1.getId()) + "\n");
        Error::log(std::to_string(getAgents().size()) + "\n");
        Error::log("Agent" + std::to_string(a1.getId()) + ": " + getAgents()[a1.getId()]->getLocation().toString() + " --> " + a1.getGoal()->toString() + "\n");
    }
    // Error::log("Heuristic cost between starting points is "
    //  + std::to_string(getGraph()->heuristicCost(
    //      getGraph()->getVertices()[37],
    //      getGraph()->getVertices()[38]))
    //  + "\n");

    Solution solution = HighLevelCBS::get_instance()
        .findSolution(getGraph(), agentInfos, LowLevelCBS::get_instance());
    
    // Distribute paths
    distributeSolution(solution);

    return true;
}

void ExperimentData::distributeSolution(Solution solution){
    for (auto agent : getAgents()){
        agent->getBot()->setPath(solution.paths[agent->getId()]);
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