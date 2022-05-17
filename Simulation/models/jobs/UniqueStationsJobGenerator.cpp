

#include "UniqueStationsJobGenerator.hpp"

#include <vector>

UniqueStationsJobGenerator::UniqueStationsJobGenerator(int numOfStations, std::set<int> endStations, int numOfJobs, int numOfEndStations) :
    JobGenerator(numOfStations, endStations, numOfJobs) {
    eng = std::mt19937(0); // seed the generator

    availableStations = {};
    for (int i = numOfEndStations; i < numOfStations; i++){
        availableStations.push_back(i);
    }

    distrEnd = std::uniform_int_distribution<>(2, 3);  // define the range of how many stations to visit
}

std::unique_ptr<Job> UniqueStationsJobGenerator::generateJob() {
    long unsigned int amountPickups = distrEnd(eng);
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
    distr = std::uniform_int_distribution<>(0, stations.size()-1);
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