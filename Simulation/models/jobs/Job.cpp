#include "Job.hpp"

#include <exception>

Job::Job(std::set<int> stationsToVisit, std::set<int> endStations) :
    JobBlueprint(std::move(stationsToVisit), std::move(endStations)) {}


std::set<int> Job::getRemainingStations() {
    return stationsToVisit;
}

void Job::visitedStation(int stationID) {
    if(stationsToVisit.find(stationID) != stationsToVisit.end())
        stationsToVisit.erase(stationID);
    else if(stationsToVisit.empty() && isEndStation(stationID))
        completed = true;
    else
        throw std::invalid_argument("Tried to remove stations not part of the station plan.");
}
