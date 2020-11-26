#ifndef SWARMSIMULATOR_JOB_HPP
#define SWARMSIMULATOR_JOB_HPP

#include "JobBlueprint.hpp"

#include <set>
#include <functional>

class Job : public JobBlueprint {
public:
    Job(std::set<int> stationsToVisit, std::set<int> endStations, std::function<void()> callBack);

    std::set<int> getRemainingStations() override;
    void visitedStation(int stationID) override;
    void markAsCompleted() override;

private:
    std::function<void()> callBackFunction;
};


#endif //SWARMSIMULATOR_JOB_HPP
