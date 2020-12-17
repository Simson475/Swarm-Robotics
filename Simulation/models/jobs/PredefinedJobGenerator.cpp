

#include "PredefinedJobGenerator.hpp"

PredefinedJobGenerator::PredefinedJobGenerator(int numOfStations, std::set<int> endStations, int numOfJobs) :
JobGenerator(numOfStations, endStations, numOfJobs) {
    eng = std::mt19937(0); // seed the generator
    distr = std::uniform_int_distribution<>(numOfEndStations, numOfStations - 1); // define the range of stations ids
    distrEnd = std::uniform_int_distribution<>(2, 4);  // define the range of how many stations to visit
}

std::unique_ptr<Job> PredefinedJobGenerator::generateJob() {
    int amountPickups = distrEnd(eng);
    std::set<int> stationsToVisit{};

    while (stationsToVisit.size() < (long unsigned int)amountPickups){
        stationsToVisit.insert(distr(eng));
    }

    jobsGenerated++;
    return std::make_unique<Job>(Job{getNextJobID(), stationsToVisit, endStations, std::bind(&JobGenerator::completedJob, this)});
}

std::unique_ptr<Job> PredefinedJobGenerator::getNextJob() {
    return this->generateJob();
}