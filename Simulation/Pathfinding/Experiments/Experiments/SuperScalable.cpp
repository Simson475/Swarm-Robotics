#include "HighLevelCBS.hpp"
#include <thread>
#include <future>
int maxTime = 10;
int maxLoops = 100;
int threads = 8;
static int counter = 0;
static int failures = 0;
std::vector<bool> doneThreads(threads, false);

std::shared_ptr<Graph> generateGraph(int size)
{
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
    return std::make_shared<Graph>(vertices);
}
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
int64_t tempFunc(int size, std::vector<AgentInfo> agents, int maxTime, int threadNumber)
{
    // create new object using copy constructor constructor
    std::shared_ptr<Graph> copiedGraph = generateGraph(size);
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
        return maxTime * microInMinutes * 10;
    }
}

int64_t getResult(int threads, int size, std::vector<std::vector<AgentInfo>> agents, std::vector<std::shared_ptr<Vertex>> vertices, int maxTime, int loops)
{
    unsigned long long fullCount = 0;
    int startCounter = 0;
    int doneCounter = 0;
    std::vector<std::future<int64_t>> futures(threads);
    for (int i = 0; i < threads; i++)
    {
        startCounter++;
        futures[i] = std::async(std::launch::async, tempFunc, size, agents[i], maxTime,i);
    }
    while (doneCounter < loops)
    {
        for (int i = 0; i < threads; i++)
        {
            if (doneThreads[i])
            {
                auto result = futures[i].get();
                doneCounter++;
                fullCount += result;
                doneThreads[i]=false;
                if (startCounter < loops)
                {
                    startCounter++;
                    futures[i] = std::async(std::launch::async, tempFunc, size, agents[i], maxTime,i);
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

    for (int size = 3; size <= 10; size++)
    {

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

        for (int agentCount = 1; agentCount <= 12; ++agentCount)
        {
            counter = 0;
            failures = 0;
            unsigned long long timeSpent = 0;
            std::chrono::steady_clock::time_point experimentBeginTime = std::chrono::steady_clock::now();

            std::cout << "Running experiment with " << agentCount << " agents.. size: " << width << "\n";
            std::cout.flush();
            std::string experimentResultFile = experimentResultDir + "/SuperScalable.txt";
            // Remove any existing results if they exist
            // remove(&experimentResultFile[0]);

            logger.setLogFile(experimentResultFile);
            // Arrange experiment
            auto agents = generateAgentInfo(threads, agentCount, vertices, width * height);
            // Act

            timeSpent += getResult(threads, size, agents, vertices, maxTime, maxLoops);

            // When plotting in pgfplots, the first element becomes the x-axis, 2. becomes the y-axis and the 3. one becomes the z-axis
            if (agentCount == 1){
                agentCount+=10;
                continue;
            }   
            auto result = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experimentBeginTime).count();
            std::cout << "fullTime: " << result << " \n";

            (*logger.begin()) << "" << agentCount << " " << width << " " << timeSpent / maxLoops << " " << failures << "\n";
            logger.end();
            std::cout << "Done\n";
        }
        (*logger.begin()) << "\n";
        logger.end();
    }
}