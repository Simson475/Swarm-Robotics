#include "HighLevelCBS.hpp"
#include <thread>
#include <future>
#include <sys/stat.h>

int maxTime = 10;
int maxLoops = 100;
int threads = 7;
static int counter = 0;
static int failures = 0;
std::vector<bool> doneThreads(threads, false);

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
int getAvailableStation(std::vector<int> &stations)
{
    int stationIndex = rand() % stations.size();
    int station = stations[stationIndex];
    stations.erase(stations.begin() + stationIndex);
    return station;
}

std::vector<AgentInfo> generateAgentInfo(int threads, int agentCount, std::vector<std::shared_ptr<Vertex>> vertices, int verticeCount)
{
    std::vector<AgentInfo> agents({(long unsigned int)agentCount});

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
int64_t tempFunc(int size, int maxTime, int threadNumber, std::shared_ptr<Graph> copiedGraph, std::vector<AgentInfo> agents)
{
    // create new object using copy constructor constructor
    auto highLevelCBS = HighLevelCBS{};
    auto lowLevelCBS = LowLevelCBS{};
    try
    {
        std::chrono::steady_clock::time_point experimentBeginTime = std::chrono::steady_clock::now();
        Solution solution = highLevelCBS.findSolution(copiedGraph, agents, lowLevelCBS, 0, maxTime);
        auto result = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experimentBeginTime).count();
        counter++;
        std::cout << counter << " is done \n";
        doneThreads[threadNumber] = true;
        return result;
    }
    catch (std::string error)
    {
        counter++;
        std::cout << counter << " is done (took too long)\n";
        int microInMinutes = 6e+7;
        failures++;
        doneThreads[threadNumber] = true;
        return maxTime * microInMinutes * 2;
    }
}

int64_t getResult(int threads, int size, int maxTime, int loops, int agentCount)
{
    unsigned long long fullCount = 0;
    int startCounter = 0;
    int doneCounter = 0;
    std::vector<std::future<int64_t>> futures(threads);
    for (int i = 0; i < threads; i++)
    {
        startCounter++;
        std::shared_ptr<Graph> copiedGraph = generateGraph(size);
        auto agents = generateAgentInfo(threads, agentCount, copiedGraph->getVertices(), size*size);
        futures[i] = std::async(std::launch::async, tempFunc, size, maxTime, i, copiedGraph, agents);
    }
    while (doneCounter < loops)
    {
        for (int i = 0; i < threads; i++)
        {
            if (doneThreads[i])
            {
                doneThreads[i]=false;
                auto result = futures[i].get();
                doneCounter++;
                fullCount += result;
                if (startCounter < loops)
                {
                    startCounter++;
                    std::shared_ptr<Graph> copiedGraph = generateGraph(size);
                    auto agents = generateAgentInfo(threads, agentCount, copiedGraph->getVertices(), size*size);
                    futures[i] = std::async(std::launch::async, tempFunc, size, maxTime, i, copiedGraph, agents);
                }
            }
        }
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

    for (int size = 6; size <= 20; size+=2)
    {

        for (int agentCount = 1; agentCount <= 13; ++agentCount)
        {
            counter = 0;
            failures = 0;
            std::cout << "Running experiment with " << agentCount << " agents.. size: " << size << "\n";
            std::cout.flush();
            std::string experimentResultFile = experimentResultDir + "/SuperScalable.txt";
            logger.setLogFile(experimentResultFile);
            unsigned long long timeSpent = getResult(threads, size, maxTime, maxLoops, agentCount);
            //warmup which is apparently needed
            if (agentCount == 1){
                continue;
            }   

            (*logger.begin()) << "" << agentCount << " " << size << " " << timeSpent / maxLoops << " " << failures << "\n";
            logger.end();
        }
        (*logger.begin()) << "\n"; logger.end();
    }
}