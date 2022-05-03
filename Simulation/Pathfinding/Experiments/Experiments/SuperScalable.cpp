#include "HighLevelCBS.hpp"
#include <thread>
#include <future>

static int counter = 0;

int getAvailableStation(std::vector<int> &stations)
{
    int stationIndex = rand() % stations.size();
    int station = stations[stationIndex];
    stations.erase(stations.begin() + stationIndex);
    return station;
}

std::vector<std::vector<AgentInfo>> generateAgentInfo(int threads, int agentCount, std::vector<std::shared_ptr<Vertex>> vertices, int verticeCount)
{
    std::vector<std::vector<AgentInfo>> agents(threads, std::vector<AgentInfo>({(long unsigned int)agentCount}));
    for (int j = 0; j < threads; j++)
    {
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
            agents[j][i] = AgentInfo(i, Action(0, startVertex, startVertex, 0), goalVertex);
        }
    }
    return agents;
}
int64_t tempFunc(std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents)
{
    // create new object using copy constructor constructor
    std::shared_ptr<Graph> copiedGraph = std::make_shared<Graph>(*graph);
    auto highLevelCBS = HighLevelCBS{};
    auto lowLevelCBS = LowLevelCBS{};
    try
    {
        std::chrono::steady_clock::time_point experimentBeginTime = std::chrono::steady_clock::now();
        Solution solution = highLevelCBS.findSolution(copiedGraph, agents, lowLevelCBS);
        auto result = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experimentBeginTime).count();
        counter++;
        std::cout << counter << " is done \n";
        return result;
    }
    catch (std::string error)
    {
        return INT64_MAX;
    }
}
static int failures = 0;

int64_t getResult(int threads, std::shared_ptr<Graph> graph, std::vector<std::vector<AgentInfo>> agents, std::vector<std::shared_ptr<Vertex>> vertices)
{
    auto fullCount = 0;

    std::vector<std::future<int64_t>> futures(threads);
    for (int i = 0; i < threads; i++)
    {
        futures[i] = std::async(std::launch::async, tempFunc, graph, agents[i]);
    }
    for (int i = 0; i < threads; i++)
    {
        auto result = futures[i].get();
        if (result == INT64_MAX)
        {
            failures++;
            std::cout << "Failure\n";
            continue;
        }
        fullCount += result;
    }
    return fullCount;
}

int main(int argc, char *argv[])
{
    srand(time(NULL));
    Logger &logger = Logger::get_instance();
    // Create folder for results
    std::string experimentResultDir = "SuperScalable_experiment_result";
    mkdir(&experimentResultDir[0], 0777);

    for (int size = 1; size <= 10; size++)
    {
        int threads = 10;
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
        auto graph = std::make_shared<Graph>(vertices);

        for (int agentCount = 1; agentCount <= 10; ++agentCount)
        {
            counter = 0;
            if (agentCount > width * height / 2)
                continue;
            auto timeSpent = 0;
            int maxLoops = 100;
            std::chrono::steady_clock::time_point experimentBeginTime = std::chrono::steady_clock::now();

            for (int loops = 0; loops < maxLoops; loops += threads)
            {
                std::cout << "Running experiment with " << agentCount << " agents.. size: " << width << "\n";
                std::cout.flush();
                std::string experimentResultFile = experimentResultDir + "/SuperScalable.txt";
                // Remove any existing results if they exist
                // remove(&experimentResultFile[0]);

                logger.setLogFile(experimentResultFile);
                // Arrange experiment
                auto agents = generateAgentInfo(threads, agentCount, vertices, width * height);
                // Act

                timeSpent += getResult(threads, graph, agents, vertices);
                if (failures != 0 && loops+threads >= maxLoops)
                {
                    auto newAgents = generateAgentInfo(failures, agents.size(), vertices, vertices.size());
                    timeSpent += getResult(failures, graph, newAgents, vertices);
                    failures = 0;

                }
            }
            // When plotting in pgfplots, the first element becomes the x-axis, 2. becomes the y-axis and the 3. one becomes the z-axis
            if (agentCount == 1)
                continue;
            auto result = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experimentBeginTime).count();
            std::cout << "fullTime: " << result<< " \n";

            (*logger.begin()) << "" << agentCount << " " << width << " " << timeSpent / maxLoops << "\n";
            logger.end();
            std::cout << "Done\n";
        }
        (*logger.begin()) << "\n";
        logger.end();
    }
}