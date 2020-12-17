#include "Job.hpp"

#include <exception>

Job::Job(int id, std::set<int> stationsToVisit, std::set<int> endStations, std::function<void()> callBack) :
    JobBlueprint(id, std::move(stationsToVisit), std::move(endStations)) {
    this->callBackFunction = callBack;
}


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

void Job::markAsCompleted(){
    callBackFunction();
}
