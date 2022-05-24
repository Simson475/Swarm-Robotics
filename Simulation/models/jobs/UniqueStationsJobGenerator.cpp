

#include "UniqueStationsJobGenerator.hpp"

#include <vector>

UniqueStationsJobGenerator::UniqueStationsJobGenerator(int numOfStations, std::set<int> endStations, int numOfJobs, int numOfEndStations) :
    JobGenerator(numOfStations, endStations, numOfJobs) {
    
    int seed = 67891234;
    try {
        argos::TConfigurationNode &t_node = argos::CSimulator::GetInstance().GetConfigurationRoot();
        argos::TConfigurationNode &params = argos::GetNode(t_node, "experiment_settings");
        argos::GetNodeAttribute(params, "seed", seed);
    }
    catch (argos::CARGoSException &e){
        std::cerr << "Job generator seed defaulted to: " << seed << std::endl;
    }

    eng = std::mt19937(seed); // seed the generator

    availableStations = {};
    for (int i = numOfEndStations; i < numOfStations; i++){
        availableStations.push_back(i);
    }

}

std::unique_ptr<Job> UniqueStationsJobGenerator::generateJob() {
    long unsigned int amountPickups = 3;
    std::set<int> stationsToVisit{};

    while (stationsToVisit.size() < amountPickups){
        Error::log(std::to_string(availableStations.size()) + " aStations\n");
        stationsToVisit.emplace(getAvailableStation(availableStations, eng));
    }

    jobsGenerated++;
    return std::make_unique<Job>(Job{getNextJobID(), stationsToVisit, endStations, std::bind(&JobGenerator::completedJob, this)});
}

std::unique_ptr<Job> UniqueStationsJobGenerator::getNextJob() {
    return this->generateJob();
}

int UniqueStationsJobGenerator::getAvailableStation(std::vector<int> &stations, std::mt19937 eng)
{
    distr = std::uniform_int_distribution<>(0, stations.size() - 1);
    int stationIndex = distr(eng);
    int station = stations[stationIndex];
    stations.erase(stations.begin() + stationIndex);
    return station;
}

void UniqueStationsJobGenerator::workedAtStation(int stationId){
    Error::log(std::to_string(stationId) + " worked at \n");
    if (endStations.find(stationId) == endStations.end()){
        availableStations.push_back(stationId);
    }
    else{
        Error::log(std::to_string(stationId) + " is and end station \n");
    }
}