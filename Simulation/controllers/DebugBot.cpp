

#include "DebugBot.hpp"
#include <regex>
#include <filesystem>

/*
 * The DebugBot will look for a file in the folder "plans" in the current dir. In that folder,
 * there must be a file on the form "<ID>_plans.csv".
 * If not, an exception is thrown.
 */

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

    if (!plan_file)
        throw std::runtime_error("Could not find plan file for debug bot with ID: " + m_strId);

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