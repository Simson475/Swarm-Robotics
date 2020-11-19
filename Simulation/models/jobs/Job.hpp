#ifndef SWARMSIMULATOR_JOB_HPP
#define SWARMSIMULATOR_JOB_HPP

#include "JobBlueprint.hpp"

#include <set>


class Job : public JobBlueprint {
public:
    Job(std::set<int> stationsToVisit, std::set<int> endStations);

    std::set<int> getRemainingStations() override;
    void visitedStation(int stationID) override;

};


#endif //SWARMSIMULATOR_JOB_HPP
