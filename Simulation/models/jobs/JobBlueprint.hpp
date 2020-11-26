#ifndef SWARMSIMULATOR_JOBBLUEPRINT_HPP
#define SWARMSIMULATOR_JOBBLUEPRINT_HPP

#include <set>
#include <exception>

class JobBlueprint {
public:

    JobBlueprint(std::set<int> stationsToVisit, std::set<int> endStations) :
        stationsToVisit(std::move(stationsToVisit)),
        endStations(std::move(endStations)) {}

    virtual std::set<int> getRemainingStations(){
        throw std::logic_error("Calls base function 'getRemainingStations' of JobBlueprint.");
    }

    virtual void visitedStation(int stationID){
        throw std::logic_error("Calls base function 'visitedStation' of JobBlueprint.");
    };

    virtual void markAsCompleted(){
        throw std::logic_error("Calls base function 'markAsCompleted' of JobBlueprint.");
    }

    virtual ~JobBlueprint() = default;

    // If there are no more stations to visit, we see if the station is an end stations.
    // If there are more stations left to visit, the station must be in that set of stations.
    virtual bool isStationInJob(int stationID){
        if(stationsToVisit.empty())
            return endStations.find(stationID) != endStations.end();
        else
            return stationsToVisit.find(stationID) != stationsToVisit.end();
    }

    virtual bool isCompleted() {
        return completed;
    }

    std::set<int> getEndStations(){
        return endStations;
    }

protected:
    std::set<int> stationsToVisit;
    const std::set<int> endStations;

    bool completed = false;

    // Common helper functions
    bool isEndStation(int stationID) {
        return endStations.find(stationID) != endStations.end();
    }

};

#endif //SWARMSIMULATOR_JOBBLUEPRINT_HPP
