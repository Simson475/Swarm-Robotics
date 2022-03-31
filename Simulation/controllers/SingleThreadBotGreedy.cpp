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
#include <iterator>




std::vector<int> SingleThreadBotGreedy::constructStationPlan() {

    // First the current remaining stations in the order are sorted
    std::vector<int> tempPlan{};
    if (!currentJob->getRemainingStations().empty()) {
        for (auto &job : currentJob->getRemainingStations())
            tempPlan.push_back(job);
        sortJob(sMap.getRealShortestDistanceMatrix(), tempPlan);
    } else {
        for (auto &job : currentJob->getEndStations())
            tempPlan.push_back(job);
        sortJob(sMap.getRealShortestDistanceMatrix(), tempPlan);
        tempPlan = std::vector<int>{tempPlan.front()};
    }

    // Then a path plan that include waypoints are made:
    std::vector<int> finegrainedPlan{};
    int srcPoint = lastLocation;

    for(unsigned long station = 0; station < tempPlan.size(); station++){
        auto pointsBetweenStations = sMap.findPath(srcPoint, tempPlan.at(station));
        std::vector<int> pathBetweenStations{};
        for (auto &p : pointsBetweenStations) {
            pathBetweenStations.emplace_back(p.getId());
        }

        finegrainedPlan.insert(finegrainedPlan.end(), pathBetweenStations.begin(), pathBetweenStations.end());
        srcPoint = tempPlan.at(station);
    }

    //Now we remove all elements that are not a station ID.

    std::vector<int> stationIDs = sMap.stationIDs;
    stationIDs.insert(stationIDs.end(), sMap.endStationIDs.begin(), sMap.endStationIDs.end());
    finegrainedPlan.erase(std::remove_if(finegrainedPlan.begin(),
        finegrainedPlan.end(),
        [stationIDs](int stationID){return std::find(stationIDs.begin(), stationIDs.end(), stationID) == stationIDs.end();}),
                          finegrainedPlan.end());

    return finegrainedPlan;
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


REGISTER_CONTROLLER(SingleThreadBotGreedy, "SingleThreadBotGreedy")