

#include "JobGenerator.hpp"

#include <random>
#include <set>
#include <exception>

JobGenerator::JobGenerator(int numOfStations, std::set<int> endStations, int numOfJobs) :
    numOfStations(numOfStations),
    endStations(endStations),
    numOfEndStations((int)endStations.size()),
    numOfJobs(numOfJobs){}

// This function is hardcored in the way that end stations have the first IDs in the map and the stations have the
// next IDs. After the stations, the waypoints gets IDs.
std::unique_ptr<Job> JobGenerator::generateJob() {
    std::random_device rd;  // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_int_distribution<> distr(numOfEndStations, numOfStations - 1);   // define the range of stations ids
    std::uniform_int_distribution<> distrEnd(2, 4); // define the range of how many stations to visit

    int amountPickups = distrEnd(eng);
    std::set<int> stationsToVisit{};

    while ((int)stationsToVisit.size() < amountPickups){
        stationsToVisit.insert(distr(eng));
    }

    jobsGenerated++;
    return std::make_unique<Job>(Job{getNextJobID(), stationsToVisit, endStations, std::bind(&JobGenerator::completedJob, this)});
}

int JobGenerator::getNextJobID(){
    return jobID++;
}

std::unique_ptr<Job> JobGenerator::generateGetHomeJob(int location){
    std::set<int> stationsToVisit{};
    std::set<int> endLocation{};
    endLocation.insert(location);

    return std::make_unique<Job>(Job{-1, stationsToVisit, endLocation, [](){}});
}

bool JobGenerator::anyJobsLeft() {
    return numOfJobs > jobsGenerated;
}

bool JobGenerator::allJobsCompleted() {
    return numOfJobs - jobsCompleted == 0;
}

std::unique_ptr<Job> JobGenerator::getNextJob() {
    return generateJob();
}

void JobGenerator::completedJob() {
    jobsCompleted++;
    int simTime = argos::CSimulator::GetInstance().GetSpace().GetSimulationClock();
    lastCompletedJobSimTime = simTime;
    
    std::ofstream out;
    out.open(std::string{std::filesystem::current_path()} + "/jobProgress.txt", std::ofstream::app);
    out << jobsCompleted << " " << simTime << "\n";
    out.close();


    if (jobsCompleted == 700){
        exit(1);
    }
    if(jobsCompleted > numOfJobs)
        throw std::logic_error("More completed jobs than jobs generated");
}

void JobGenerator::workedAtStation(int stationId){
    // Do nothing
}