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
    for (size_t i = 0; i < job.size() - 1; i++) {
        int k = i;
        float min = std::numeric_limits<float>::infinity();
        for (size_t j = i; j < job.size() - 1; j++) {
            float temp;
            if (i == 0) {
                temp = shortestDistances[lastLocation][job[j]];
            } else temp = shortestDistances[job[i - 1]][job[j]];

            if (temp < min) {
                min = temp;
                k = j;
            }
        }
        std::swap(job[i], job[k]);
    }
}


REGISTER_CONTROLLER(SingleThreadBotGreedy, "SingleThreadBotGreedy_controller")