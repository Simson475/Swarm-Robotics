

#include "DebugBot.hpp"
#include <regex>
#include <filesystem>


void DebugBot::specialInit() {
    getPlans();
}

std::vector<int> DebugBot::constructStationPlan(){
    auto next_plan = stationPlans.front();
    stationPlans.pop();
    return next_plan;
}

std::vector<int> DebugBot::constructWaypointPlan(){
    auto next_plan = waypointPlans.front();
    waypointPlans.pop();
    return next_plan;
}

void DebugBot::getPlans(){
    std::ifstream plan_file{std::string{std::filesystem::current_path()} + "/plans/" + GetId() + "_plans.csv"};

    std::string line;
    while(std::getline(plan_file, line)) {
        if(isStationPlan(line))
            stationPlans.push(extractPlanFromLine(line));
        else
            waypointPlans.push(extractPlanFromLine(line));
    }
}


bool DebugBot::isStationPlan(std::string line) {
    return line.find("Station") != std::string::npos;
}

std::vector<int> DebugBot::extractPlanFromLine(std::string line) {
    std::string plan = line.substr(line.find(";") + 1, line.size() - 1);

    std::vector<int> points{};

    std::regex rx(R"(\d+)");
    std::smatch m;
    std::string str = plan;
    while (regex_search(str, m, rx)) {
        points.push_back(std::stoi(m[0]));
        str = m.suffix().str();
    }

    return points;
}

REGISTER_CONTROLLER(DebugBot, "DebugBot")