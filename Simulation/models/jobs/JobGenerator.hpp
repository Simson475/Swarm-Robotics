#ifndef SWARMSIMULATOR_JOBGENERATOR_HPP
#define SWARMSIMULATOR_JOBGENERATOR_HPP

#include "Job.hpp"

#include <set>
#include <vector>


class JobGenerator {
private:
    const int numOfStations;
    const std::set<int> endStations;
    const int numOfEndStations;
    const int numOfJobs;

    int jobsGenerated = 0;
    int jobsCompleted = 0;

    std::vector<Job> jobs{};

    Job generateJob();



public:
    JobGenerator(int numOfStations, std::set<int> endStations, int numOfJobs);

    bool anyJobsLeft();
    bool allJobsCompleted();
    Job getNextJob();
    void completedJob();
};


#endif //SWARMSIMULATOR_JOBGENERATOR_HPP
