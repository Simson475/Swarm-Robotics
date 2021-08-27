

#include "PredefinedDescreteJobGenerator.hpp"

#include <vector>

PredefinedDescreteJobGenerator::PredefinedDescreteJobGenerator(int numOfStations, std::set<int> endStations, int numOfJobs) :
    JobGenerator(numOfStations, endStations, numOfJobs) {
    eng = std::mt19937(0); // seed the generator

    // Push zero probability for the end stations
    std::vector<int> probabilities{};
    for(unsigned long i = 0; i < endStations.size(); i++){
        probabilities.push_back(0);
    }

    // Push increasing probabilities for other stations.
    for(unsigned long i = 1; i <= numOfStations - endStations.size(); i++){
        probabilities.push_back(i);
    }


    distr = std::discrete_distribution<int>(probabilities.begin(), probabilities.end()); // define the range of stations ids
    distrEnd = std::uniform_int_distribution<>(2, 4);  // define the range of how many stations to visit
}

std::unique_ptr<Job> PredefinedDescreteJobGenerator::generateJob() {
    long unsigned int amountPickups = distrEnd(eng);
    std::set<int> stationsToVisit{};

    while (stationsToVisit.size() < amountPickups){
        stationsToVisit.insert(distr(eng));
    }

    jobsGenerated++;
    return std::make_unique<Job>(Job{getNextJobID(), stationsToVisit, endStations, std::bind(&JobGenerator::completedJob, this)});
}

std::unique_ptr<Job> PredefinedDescreteJobGenerator::getNextJob() {
    return this->generateJob();
}