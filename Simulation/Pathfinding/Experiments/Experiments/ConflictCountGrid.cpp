#include "HighLevelCBS.hpp"
#include <sys/stat.h>

int getAvailableStation(std::vector<int> &stations)
{
    int stationIndex = rand() % stations.size();
    int station = stations[stationIndex];
    stations.erase(stations.begin() + stationIndex);
    return station;
}
std::vector<AgentInfo> generateAgentInfo(int agentCount, std::vector<std::shared_ptr<Vertex>> vertices)
{
    std::vector<AgentInfo> agents({(long unsigned int)agentCount});
    int verticeCount = vertices.size();
    std::vector<int> freeStarts(verticeCount);
    std::vector<int> freeGoals(verticeCount);
    for (int i = 0; i < verticeCount; ++i)
    {
        freeStarts[i] = i;
        freeGoals[i] = i;
    }

    for (int i = 0; i < agentCount; ++i)
    {
        auto startVertex = vertices[getAvailableStation(freeStarts)];
        auto goalVertex = vertices[getAvailableStation(freeGoals)];
        agents[i] = AgentInfo(i, Action(0, startVertex, startVertex, 0), goalVertex);
    }

    return agents;
}
std::shared_ptr<Graph> generateGraph(int size)
{
    int height = size;
    int width = size;
    int verticesMade = 0;
    std::vector<std::shared_ptr<Vertex>> vertices{(long unsigned int)(width * height)};
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            vertices[verticesMade] = std::make_shared<Vertex>(verticesMade);
            if (i != 0)
            {
                float edgeCost = rand() % 50 + 50;
                ;
                vertices[verticesMade]->addEdge({std::make_shared<Edge>(vertices[verticesMade], vertices[verticesMade - width], edgeCost)});
                vertices[verticesMade - width]->addEdge({std::make_shared<Edge>(vertices[verticesMade - width], vertices[verticesMade], edgeCost)});
            }
            if (j != 0)
            {
                float edgeCost = rand() % 50 + 50;
                ;
                vertices[verticesMade]->addEdge({std::make_shared<Edge>(vertices[verticesMade], vertices[verticesMade - 1], edgeCost)});
                vertices[verticesMade - 1]->addEdge({std::make_shared<Edge>(vertices[verticesMade - 1], vertices[verticesMade], edgeCost)});
            }
            verticesMade++;
        }
    }
    return std::make_shared<Graph>(vertices);
}

int main(int argc, char *argv[])
{
    Logger &logger = Logger::get_instance();
    // Create folder for results
    std::string experimentResultDir = "ConflictCountGrid_experiment_result";
    mkdir(&experimentResultDir[0], 0777);
    auto minAgentCount = 1;
    auto maxAgentCount = 15;
    auto size = 6;
    auto seed = 165244401;
    srand(seed);
    for(auto agentCount = minAgentCount;agentCount<maxAgentCount;agentCount++){
    std::cout << "Running experiment with " << agentCount << " agents.. seed is: "<<seed<<"..";
    std::cout.flush();
    std::string experimentResultFile = experimentResultDir + "/" + std::to_string(agentCount) + "agents_ConflictCountGrid.txt";
    // Remove any existing results if they exist
    remove(&experimentResultFile[0]);
    logger.setLogFile(experimentResultFile);

    // Arrange experiment
    auto graph = generateGraph(size);
    auto vertices = graph->getVertices();
    auto agents = generateAgentInfo(agentCount, vertices);

    // Act
    std::chrono::steady_clock::time_point experimentBeginTime = std::chrono::steady_clock::now();
    Solution solution = HighLevelCBS::get_instance().findSolution(graph, agents, LowLevelCBS::get_instance());
    auto experimentTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experimentBeginTime).count();

    // Results
    (*logger.begin()) << "Experiment took " << experimentTime << "[µs]\n";
    logger.end();
    std::cout << "Done\n";
    }
}
