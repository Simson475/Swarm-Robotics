#include "HighLevelCBS.hpp"

int main(int argc, char *argv[]) {
    // Arrange
    Logger& logger = Logger::get_instance();
    // Create folder for results
    std::string experimentResultDir = "chokepoint_experiment_result";
    mkdir(&experimentResultDir[0], 0777);

    for (int agentCount = 1; agentCount <= 15; ++agentCount){
        std::cout << "Running experiment with " << agentCount << " agents..";
        std::cout.flush();
        std::string experimentResultFile = experimentResultDir + "/" + std::to_string(agentCount) + "agents_chokepoint.txt";
        // Remove any existing results if they exist
        remove(&experimentResultFile[0]);
        logger.setLogFile(experimentResultFile);

        
        // Create vertices
        int vertexCount = agentCount + 1 + agentCount;
        std::vector<std::shared_ptr<Vertex>> vertices{(long unsigned int)(vertexCount)};
        for (int i = 0; i < vertexCount; ++i){
            vertices[i] = std::make_shared<Vertex>(i);
        }

        /// Create edges (one directional)
        int chokepointIndex = agentCount;
        // Edges from agent start to chokepoint
        for (int i = 0; i < agentCount; i++){
            std::vector<std::shared_ptr<Edge>> edges;
            edges.emplace_back(std::make_shared<Edge>(vertices[i], vertices[chokepointIndex], 100));
            vertices[i]->setEdges(edges);
        }
        // Edges from chokepoint to agent goal
        std::vector<std::shared_ptr<Edge>> edges;
        for (int i = chokepointIndex + 1; i < vertexCount; i++){
            edges.emplace_back(std::make_shared<Edge>(vertices[chokepointIndex], vertices[i], 100));
        }
        vertices[chokepointIndex]->setEdges(edges);

        auto graph = std::make_shared<Graph>(vertices);

        //AgentInfo(id, action, dest)
        std::vector<AgentInfo> agents{(long unsigned int)agentCount};
        for (int i = 0; i < agentCount; ++i){
            // Agent going from a unique vertex to a unique vertex on the opposite site of the chokepoint
            agents[i] = AgentInfo(i, Action(0, vertices[i], vertices[i], 0), vertices[chokepointIndex + 1 + i]);
        }

        // Act
        std::chrono::steady_clock::time_point experimentBeginTime = std::chrono::steady_clock::now();
        Solution solution = HighLevelCBS::get_instance().findSolution(graph, agents, LowLevelCBS::get_instance());
        auto experimentTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experimentBeginTime).count();

        // Assert
        // Expected high level iterations = 
        // 2 (best conflicts two agents' constraints)
        // * 2 (best conflicts two agents' constraints) (now two agents are waiting)
        // * 2 (best conflicts two agents' constraints) (now two agents are waiting)
        // ...
        // (until k-1 agents are waiting = 2^(k-1))
        // ... now 1 agent can pass without conflict
        // for 2 agents to pass without conflict * 2^(k-2)
        // ...
        // = 2^(k-1)*2^(k-2)*...*2 = 2^(k-1 + k-2 + ... + 1) = 2^((k-1)*k/2)

        uint expectedHighLevelIterations = std::pow(2, (agentCount - 1) * agentCount / 2);
        (*logger.begin())
         << "Total high level: " << (HighLevelCBS::get_instance().iterations) << " iterations. "
         << "Expected " << expectedHighLevelIterations << " iterations.\n"
         << "Experiment took "<< experimentTime << "[µs]\n";
        logger.end();
        std::cout << "Done\n";
    }

    std::cout << "Experiments done\n";
}

