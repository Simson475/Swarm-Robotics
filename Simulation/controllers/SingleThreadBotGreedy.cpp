#include "SingleThreadBotGreedy.hpp"
#include "argos_wrapper/argos_wrapper.hpp"

#include <exception>
#include <cstdio>
#include <regex>
#include <fstream>
#include <set>
#include <iostream>
#include <filesystem>
#include <ctime>
#include <chrono>

std::vector<int> SingleThreadBotGreedy::constructStationPlan() {
    std::vector<int> tempPlan{};
    if (!currentJob->getRemainingStations().empty()) {
        for (auto &job : currentJob->getRemainingStations())
            tempPlan.push_back(job);
        sortJob(sMap.getRealShortestDistanceMatrix(), tempPlan);
    } else {
        for (auto &job : currentJob->getEndStations())
            tempPlan.push_back(job);
        sortJob(sMap.getRealShortestDistanceMatrix(), tempPlan);
    }
    return tempPlan;
}

std::vector<int> SingleThreadBotGreedy::constructWaypointPlan() {
    std::vector<int> tempPlan{};

    auto plan = sMap.findPath(lastLocation, stationPlan.front());

    for (auto &p : plan) {
        tempPlan.emplace_back(p.getId());
    }

    return tempPlan;
}

void SingleThreadBotGreedy::sortJob(const std::vector<std::vector<float>> &shortestDistances, std::vector<int> &job) {
    std::set<int> stations{};
    std::vector<int> newPlan{};

    for (auto station : job){
        stations.insert(station);
    }

    int location = lastLocation;
    while(!stations.empty()){
        int tmp_station = 0;
        double tmp_dist = std::numeric_limits<double>::infinity();
        for (auto station : stations){
            if(shortestDistances[location][station] < tmp_dist){
                tmp_dist = shortestDistances[location][station];
                tmp_station = station;
            }
        }
        location = tmp_station;
        newPlan.push_back(tmp_station);
        stations.erase(tmp_station);
    }
    job = newPlan;
}


REGISTER_CONTROLLER(SingleThreadBotGreedy, "SingleThreadBotGreedy_controller")