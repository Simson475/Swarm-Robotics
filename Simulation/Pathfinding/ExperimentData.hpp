#ifndef EXPERIMENT_DATA_HPP
#define EXPERIMENT_DATA_HPP

#include <memory>
#include <vector>
#include "Graph.hpp"
#include "MapStructureGraph.hpp"
#include "Agent.hpp"
#include "AgentInfo.hpp"
#include "HighLevelCBS.hpp"
#include "Solution.hpp"
#include "Path.hpp"
#include "Action.hpp"

#include "Debugging.hpp"
#include "Logger.hpp"

class ExperimentData{
public:
    static ExperimentData &get_instance() {
        static ExperimentData instance;
        return instance;
    }
    std::shared_ptr<Graph> getGraph();
    std::vector<std::shared_ptr<Agent>> getAgents();
    std::vector<AgentInfo> getAgentsInfo();
    bool requestSolution(int agentId);
    int getNextStation();
private:
    std::shared_ptr<Graph> graph;
    std::shared_ptr<MapStructureGraph> mapStructureGraph;
    std::vector<std::shared_ptr<Agent>> agents;
    void distributeSolution(Solution solution);
    int nextStation = 0;
    std::vector<int> stations;
    std::vector<int> getStations();
};

#endif