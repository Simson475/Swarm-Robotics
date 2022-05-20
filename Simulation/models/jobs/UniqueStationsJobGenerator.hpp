

#ifndef UNIQUE_STATION_JOB_GENERATOR_HPP
#define UNIQUE_STATION_JOB_GENERATOR_HPP

#include "JobGenerator.hpp"
#include "Job.hpp"

#include <random>
#include <list>

#include "../../Pathfinding/Debugging.hpp"

class UniqueStationsJobGenerator : public JobGenerator  {
public:
    UniqueStationsJobGenerator(int numOfStations, std::set<int> endStations, int numOfJobs, int numOfEndStations);
    std::unique_ptr<Job> getNextJob() override;
    void workedAtStation(int stationId) override;

protected:
    std::unique_ptr<Job> generateJob() override;
    int getAvailableStation(std::vector<int> &stations, std::mt19937 eng);

private:
    std::vector<int> availableStations;
    std::mt19937 eng; //Using the same seed all the time.
    std::uniform_int_distribution<> distr;
    std::uniform_int_distribution<> distrEnd;
};


#endif //UNIQUE_STATION_JOB_GENERATOR_HPP
