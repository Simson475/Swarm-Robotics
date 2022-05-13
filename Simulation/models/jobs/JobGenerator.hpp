#ifndef SWARMSIMULATOR_JOBGENERATOR_HPP
#define SWARMSIMULATOR_JOBGENERATOR_HPP

#include "Job.hpp"

#include <set>
#include <vector>
#include <memory>
#include "argos3/core/control_interface/ci_controller.h"
#include "argos3/core/simulator/loop_functions.h"
#include <fstream>
#include <iostream>
#include <cstdio>
#include <filesystem>

class JobGenerator {
protected:
    const int numOfStations;
    const std::set<int> endStations;
    const int numOfEndStations;
    const int numOfJobs;
    int jobID = 0;

    int getNextJobID();

    int jobsGenerated = 0;
    int jobsCompleted = 0;

    std::vector<Job> jobs{};

    virtual std::unique_ptr<Job> generateJob();



public:
    JobGenerator(int numOfStations, std::set<int> endStations, int numOfJobs);

    virtual ~JobGenerator() = default;

    bool anyJobsLeft();
    bool allJobsCompleted();
    virtual std::unique_ptr<Job> getNextJob();
    std::unique_ptr<Job> generateGetHomeJob(int location);
    void completedJob();
};


#endif //SWARMSIMULATOR_JOBGENERATOR_HPP
