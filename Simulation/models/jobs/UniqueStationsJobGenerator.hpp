

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
    int getAvailableStation(std::vector<int> &stations);

private:
    std::vector<int> availableStations;
    std::uniform_int_distribution<> distr;
    std::uniform_int_distribution<> distrEnd;
    std::mt19937 eng;
};


#endif //UNIQUE_STATION_JOB_GENERATOR_HPP
