#include "HighLevelCBS.hpp"
#include <thread>
#include <future>

int getAvailableStation(std::vector<int> &stations)
{
    int stationIndex = rand() % stations.size();
    int station = stations[stationIndex];
    stations.erase(stations.begin() + stationIndex);
    return station;
}

uint64_t tempFunc(std::shared_ptr <Graph> graph, std::vector<AgentInfo> agents){
    std::chrono::steady_clock::time_point experimentBeginTime = std::chrono::steady_clock::now();
    Solution solution = HighLevelCBS::get_instance().findSolution(graph, agents, LowLevelCBS::get_instance());
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experimentBeginTime).count();

}

int main(int argc, char *argv[])
{
    srand(time(NULL));
    Logger &logger = Logger::get_instance();
    // Create folder for results
    std::string experimentResultDir = "SuperScalable_experiment_result";
    mkdir(&experimentResultDir[0], 0777);

    for (int size = 1; size <= 20; size++)
    {
        int threads = 100;
        int height = 2 * size;
        int width = 2 * size;
        int verticesMade = 0;
        std::vector<std::shared_ptr<Vertex>> vertices{(long unsigned int)(width * height)};
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                vertices[verticesMade] = std::make_shared<Vertex>(verticesMade);
                if (i != 0)
                {
                    float edgeCost = rand() % 50 + 50;;
                    vertices[verticesMade]->addEdge({std::make_shared<Edge>(vertices[verticesMade], vertices[verticesMade - width], edgeCost)});
                    vertices[verticesMade - width]->addEdge({std::make_shared<Edge>(vertices[verticesMade - width], vertices[verticesMade], edgeCost)});
                }
                if (j != 0)
                {
                    float edgeCost = rand() % 50 + 50;;
                    vertices[verticesMade]->addEdge({std::make_shared<Edge>(vertices[verticesMade], vertices[verticesMade - 1], edgeCost)});
                    vertices[verticesMade - 1]->addEdge({std::make_shared<Edge>(vertices[verticesMade - 1], vertices[verticesMade], edgeCost)});
                }
                verticesMade++;
            }
        }
        auto graph = std::make_shared<Graph>(vertices);

        for (int agentCount = 1; agentCount <= 10; ++agentCount)
        {
            if(agentCount >= width*height - 2) continue;
            int64_t timeSpent=0;
            int maxLoops =100;
            for (int loops = 0; loops < maxLoops; loops+= threads)
            {
                std::cout << "Running experiment with " << agentCount << " agents.." << loops << "\n";
                std::cout.flush();
                std::string experimentResultFile = experimentResultDir + "/SuperScalable.txt";
                // Remove any existing results if they exist
                // remove(&experimentResultFile[0]);
                logger.setLogFile(experimentResultFile);
                // Arrange experiment
                std::vector<std::vector<AgentInfo>> agents(threads, std::vector<AgentInfo>({(long unsigned int)agentCount}));
                for (int j = 0; j < threads; j++)
                {
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
                        //std::cout <<"agent"<< i <<" Start:"<< startVertex->getId()<< " Goal:"<< goalVertex->getId() << "\n";
                        agents[j][i] = AgentInfo(i, Action(0, startVertex, startVertex, 0), goalVertex);
                    }
                }

                // Act
                std::vector<std::future<uint64_t>> futures(threads);
                for (int i = 0; i < threads; i++)
                {
                    futures[i] = std::async(std::launch::async,tempFunc, graph, agents[i]);
                }
                for (int i = 0; i < threads; i++)
                {
                    timeSpent += futures[i].get();
                }
            }
            // When plotting in pgfplots, the first element becomes the x-axis, 2. becomes the y-axis and the 3. one becomes the z-axis
            if (agentCount == 1) continue;
            (*logger.begin()) << "" << agentCount << " " << width << " " << timeSpent/maxLoops << "\n";
            logger.end();
            std::cout << "Done\n";
        }
        (*logger.begin()) << "\n"; logger.end();
    }
}