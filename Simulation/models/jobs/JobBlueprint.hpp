#ifndef SWARMSIMULATOR_JOBBLUEPRINT_HPP
#define SWARMSIMULATOR_JOBBLUEPRINT_HPP

#include <set>

class JobBlueprint {
public:
    JobBlueprint(std::set<int> stationsToVisit, std::set<int> endStations) :
        stationsToVisit(std::move(stationsToVisit)),
        endStations(std::move(endStations)) {}

    virtual std::set<int> getRemainingStations();
    virtual void visitedStation(int stationID);

    virtual ~JobBlueprint() = default;

protected:
    std::set<int> stationsToVisit;
    const std::set<int> endStations;

    bool completed = false;

    // Common helper functions
    bool isEndStation(int stationID) {
        return endStations.find(stationID) != endStations.end();
    }

    bool isStationInJob(int stationID){
        return stationsToVisit.find(stationID) != stationsToVisit.end();
    }

    bool isCompleted() {
        return completed;
    }

};

#endif //SWARMSIMULATOR_JOBBLUEPRINT_HPP
