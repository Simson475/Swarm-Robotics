#include "HighLevelCBS.hpp"

int main(int argc, char *argv[]) {
    Logger& logger = Logger::get_instance();
    // Create folder for results
    std::string experimentResultDir = "same_goal_bidirectional_experiment_result";
    mkdir(&experimentResultDir[0], 0777);

    for (int agentCount = 1; agentCount <= 15; ++agentCount){
        std::cout << "Running experiment with " << agentCount << " agents..";
        std::cout.flush();
        std::string experimentResultFile = experimentResultDir + "/" + std::to_string(agentCount) + "agents_same_goal_bidirectional.txt";
        // Remove any existing results if they exist
        remove(&experimentResultFile[0]);
        logger.setLogFile(experimentResultFile);

        // Arrange experiment
        // We construct a graph like the following
        // 0 1 2   ... agentCount
        //  \|/...
        //   G
        // Create vertices
        int vertexCount = agentCount + 1;
        int goalVertexIndex = vertexCount - 1;
        std::vector<std::shared_ptr<Vertex>> vertices{(long unsigned int)(vertexCount)};
        for (int i = 0; i < vertexCount; ++i){
            vertices[i] = std::make_shared<Vertex>(i);
        }
        // Create edges
        float edgeCost = 100;
        auto goalEdges = std::vector<std::shared_ptr<Edge>>();
        for (int i = 0; i < agentCount; ++i){
            // Edge from agent starts to goal
            vertices[i]->setEdges({
                std::make_shared<Edge>(vertices[i], vertices[goalVertexIndex], edgeCost)
            });
            // Edge from goal to agent starts
            goalEdges.push_back(std::make_shared<Edge>(vertices[goalVertexIndex], vertices[i], edgeCost));
        }
        vertices[goalVertexIndex]->setEdges(goalEdges);
        // Create graph
        auto graph = std::make_shared<Graph>(vertices);
        // Create agents
        std::vector<AgentInfo> agents{(long unsigned int)agentCount};
        for (int i = 0; i < agentCount; ++i){
            auto startVertex = vertices[i];
            auto goalVertex = vertices[goalVertexIndex];
            agents[i] = AgentInfo(i, Action(0, startVertex, startVertex, 0), goalVertex);
        }

        // Act
        std::chrono::steady_clock::time_point experimentBeginTime = std::chrono::steady_clock::now();
        Solution solution = HighLevelCBS::get_instance().findSolution(graph, agents, LowLevelCBS::get_instance());
        auto experimentTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experimentBeginTime).count();

        // Results
        (*logger.begin()) << "HighLevel used "<< HighLevelCBS::get_instance().iterations << " iterations\n"; logger.end();
        (*logger.begin()) << "Experiment took "<< experimentTime << "[µs]\n"; logger.end();
        std::cout << "Done\n";
    }
}