#include "ExperimentData.hpp"

std::shared_ptr<Graph> ExperimentData::getGraph(){
    if ( ! (graph == nullptr) ) { return graph; }

    graph = std::make_shared<Graph>(Graph(Map_Structure::get_instance()));
    return graph;
}

std::vector<std::shared_ptr<Agent>> ExperimentData::getAgents(){
    if ( ! agents.empty()) { return agents; }

    auto &tBotMap = argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    std::vector<std::shared_ptr<Agent>> _agents;
    int i = 0;
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        TestController* controller = dynamic_cast<TestController*>(&(pcBot->GetControllableEntity().GetController()));
        _agents.emplace_back(std::make_shared<Agent>(Agent(i, controller)));
        i++;

    }

    agents = _agents;

    return agents;
}

bool ExperimentData::requestSolution(int agentId){
    return false;//TODO actual code.
}



// std::vector<AgentInfo> HighLevelCBS::getAgentInfo(){
//     auto agents = ExperimentData::get_instance().agents;
//     size_t agentCount = agents->size();
//     std::vector<AgentInfo> agentInfo{agentCount};
//     for(size_t i = 0; i < agentCount; ++i){
//         agentInfo[i].setId(i);
//     }
//     return agentInfo;
// }