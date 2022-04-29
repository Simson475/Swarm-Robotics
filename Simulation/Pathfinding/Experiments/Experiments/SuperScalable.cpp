#include "HighLevelCBS.hpp"

int getAvailableStation(std::vector<int> &stations)
{
    int stationIndex = rand() % stations.size();
    int station = stations[stationIndex];
    stations.erase(stations.begin() + stationIndex);
    return station;
}

int main(int argc, char *argv[])
{
    srand(time(NULL));
    Logger &logger = Logger::get_instance();
    // Create folder for results
    std::string experimentResultDir = "SuperScalable_experiment_result";
    mkdir(&experimentResultDir[0], 0777);
    int height = 20;
    int width = 20;
    std::vector<std::shared_ptr<Vertex>> vertices{(long unsigned int)(width * height)};
    float edgeCost = 100;
    int verticesMade = 0;
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            vertices[verticesMade] = std::make_shared<Vertex>(verticesMade);
            if (i != 0)
            {
                vertices[verticesMade]->addEdge({std::make_shared<Edge>(vertices[verticesMade], vertices[verticesMade - width], edgeCost)});
                vertices[verticesMade - width]->addEdge({std::make_shared<Edge>(vertices[verticesMade - width], vertices[verticesMade], edgeCost)});
            }
            if (j != 0)
            {
                vertices[verticesMade]->addEdge({std::make_shared<Edge>(vertices[verticesMade], vertices[verticesMade - 1], edgeCost)});
                vertices[verticesMade - 1]->addEdge({std::make_shared<Edge>(vertices[verticesMade - 1], vertices[verticesMade], edgeCost)});
            }
            verticesMade++;
        }
    }
    auto graph = std::make_shared<Graph>(vertices);

    for (int agentCount = 1; agentCount <= 25; ++agentCount)
    {
        for (int loops = 0; loops < 10; loops++)
        {
            std::cout << "Running experiment with " << agentCount << " agents..";
            std::cout.flush();
            std::string experimentResultFile = experimentResultDir + "/" + std::to_string(agentCount) + "agents_SuperScalable.txt";
            // Remove any existing results if they exist
            remove(&experimentResultFile[0]);
            logger.setLogFile(experimentResultFile);

            // Arrange experiment

            std::vector<AgentInfo> agents{(long unsigned int)agentCount};
            std::vector<int> freeStarts(width * height);
            std::vector<int> freeGoals(width * height);
            for (int i = 0; i < width * height; ++i)
            {
                freeStarts[i] = i;
                freeGoals[i] = i;
            }

            for (int i = 0; i < agentCount; ++i)
            {
                auto startVertex = vertices[getAvailableStation(freeStarts)];
                auto goalVertex = vertices[getAvailableStation(freeGoals)]; // TODO set to actual goal
                agents[i] = AgentInfo(i, Action(0, startVertex, startVertex, 0), goalVertex);
            }

            // Act
            std::chrono::steady_clock::time_point experimentBeginTime = std::chrono::steady_clock::now();
            Solution solution = HighLevelCBS::get_instance().findSolution(graph, agents, LowLevelCBS::get_instance());
            auto experimentTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experimentBeginTime).count();
            // Results
            (*logger.begin()) << "(" << agentCount << ", " << width << ", " << experimentTime << ")\n";
            logger.end();
            std::cout << "Done\n";
        }
    }
}