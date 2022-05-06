#include "HighLevelCBS.hpp"
#include <sys/stat.h>

int main(int argc, char *argv[]) {
    Logger& logger = Logger::get_instance();
    // Create folder for results
    std::string experimentResultDir = "blocked_goal_experiment_result";
    mkdir(&experimentResultDir[0], 0777);

    for (int agentCount = 1; agentCount <= 1; ++agentCount){
        std::cout << "Running experiment with " << agentCount << " agents..";
        std::cout.flush();
        std::string experimentResultFile = experimentResultDir + "/" + std::to_string(agentCount) + "agents_blocked_goal.txt";
        // Remove any existing results if they exist
        remove(&experimentResultFile[0]);
        logger.setLogFile(experimentResultFile);

        // Arrange experiment
        // Create vertices
        int vertexCount = 100;
        std::vector<std::shared_ptr<Vertex>> vertices(vertexCount);
        for (int i = 0; i < vertexCount; ++i){
            vertices[i] = std::make_shared<Vertex>(i);
        }
        float edgeCost = 50;
        vertices[0]->setEdges({
            std::make_shared<Edge>(vertices[0], vertices[1], edgeCost),
        });
        vertices[1]->setEdges({
            std::make_shared<Edge>(vertices[1], vertices[0], edgeCost),
            std::make_shared<Edge>(vertices[1], vertices[2], edgeCost),
        });
        for(int i = 2; i < vertexCount; ++i){
            std::vector<std::shared_ptr<Edge>> edges;
            if (i == 2) {
                edges.push_back(std::make_shared<Edge>(vertices[i], vertices[1], edgeCost));
            }
            for(int j = 2; j < vertexCount; ++j){
                if (i != j){
                    edges.push_back(std::make_shared<Edge>(vertices[i], vertices[j], edgeCost));
                }
            }
            vertices[i]->setEdges(edges);
        }
        auto graph = std::make_shared<Graph>(vertices);
        auto startVertex = vertices[2];
        auto goalVertex = vertices[0];
        AgentInfo agent = AgentInfo(0, Action(0, startVertex, startVertex, 0), goalVertex);
        Constraint constraint = Constraint(agent.getId(), Location(vertices[1]), 0, 300);

        // Act
        std::chrono::steady_clock::time_point experimentBeginTime = std::chrono::steady_clock::now();
        LowLevelCBS::get_instance().getIndividualPath(graph, agent, {constraint});
        auto experimentTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experimentBeginTime).count();

        // Results
        (*logger.begin()) << "Experiment took "<< experimentTime << "[µs]\n"; logger.end();
        std::cout << "Done\n";
    }
}

