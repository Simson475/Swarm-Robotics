//
// Created by martin on 11/11/20.
//

#include "SingleThreadUppaalBot.hpp"
#include "parsing/uppaal_model_parsing.hpp"
#include "argos_wrapper/argos_wrapper.hpp"
#include "exceptions/StrategySynthesisError.hpp"

#include <exception>
#include <cstdio>
#include <regex>
#include <fstream>
#include <set>
#include <iostream>
#include <filesystem>
#include <ctime>
#include <chrono>


void SingleThreadUppaalBot::Init(argos::TConfigurationNode &t_node){
    RobotInterface::Init(t_node);

    argos::TConfigurationNode& params = argos::GetNode(t_node, "state");
    argos::GetNodeAttribute(params, "num_of_runs", num_of_runs);
}

std::vector<int> SingleThreadUppaalBot::constructStationPlan(){
    auto t_start = std::chrono::high_resolution_clock::now();

    std::vector<int> stationPlan{};
    uint failed = 0;

    // If no strategy is found a custom exception is thrown and caught.
    while (stationPlan.empty()) {
        log_helper("Constructs Station model");
        constructStationUppaalModel(failed);
        log_helper("Constructed Station model");

        try {
            stationPlan = getStationPlan(runStationModel());
            if(stationPlan.empty())
                failed++;
        }
        catch (const StrategySynthesisError &e){
            log_helper("Caught StrategySynthesisError");
            log_helper(e.what());
            failed++;
        }

    }
    auto t_end = std::chrono::high_resolution_clock::now();
    auto time_elapsed_s = std::chrono::duration<double, std::milli>(t_end-t_start).count() / 1000;
    experiment_helper("StationPlan", time_elapsed_s, currentJob->getRemainingStations().size(), stationPlan.size());

    return stationPlan;
}

std::vector<int> SingleThreadUppaalBot::constructWaypointPlan(){
    auto t_start = std::chrono::high_resolution_clock::now();

    std::vector<int> waypointPlan{};
    uint failed = 0;

    // If no strategy is found a custom exception is thrown and caught.
    while (waypointPlan.empty()) {
        log_helper("Constructs Waypoint model");
        constructWaypointUppaalModel(failed);
        log_helper("Constructed Waypoint model");

        try {
            waypointPlan = getWaypointPlan(runWaypointModel());
            if(waypointPlan.empty())
                failed++;
        }
        catch (const StrategySynthesisError &e){
            log_helper("Caught StrategySynthesisError");
            log_helper(e.what());
            failed++;
        }
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    auto time_elapsed_s = std::chrono::duration<double, std::milli>(t_end-t_start).count() / 1000;
    experiment_helper("WaypointPlan", time_elapsed_s, 1, waypointPlan.size());

    return waypointPlan;
}



std::vector<int> SingleThreadUppaalBot::getStationPlan(std::string modelOutput) {
    std::smatch m;
    std::regex queryForm ("cur_station:\n");
    std::regex_search(modelOutput, m, queryForm);


    std::string queryResult = modelOutput.substr(m.position());
    std::ofstream debug2{std::string{std::filesystem::current_path()} + "/debug2.txt"};

    debug2 << modelOutput << "\n\n";
    debug2 << queryResult;

    std::set<int> stationsVisited{};
    std::vector<int> stationPlan{};


    std::regex queryPart(R"(([0-9]+)([.][0-9])?,([0-9]+))");
    std::regex_search(queryResult, m, queryPart);

    // We keep every other position in the sequence of steps that the robot moved through. We neglect the first
    // since it is position 0, that the robot's model starts in.
    bool keep = false;
    for (auto it = std::sregex_iterator(queryResult.begin(), queryResult.end(), queryPart);
        it != std::sregex_iterator(); it++){
        m = *it;
        debug2 << m[0] << ": ->  "  << m[3] << std::endl;

        if(keep) {
            stationPlan.push_back(std::stoi(m[3]));
        }
        keep = !keep;
    }

    debug2 << "Station plan:\n";
    for(int s : stationPlan)
        debug2 << s << std::endl;


    debug2.close();

    // We remove the first element as it is the location the robot is in.
    stationPlan.erase(stationPlan.begin());
    return stationPlan;
}

std::vector<int> SingleThreadUppaalBot::getWaypointPlan(std::string modelOutput) {
    std::smatch m;
    std::regex queryForm ("cur_station:\n");
    std::regex_search(modelOutput, m, queryForm);


    std::string queryResult = modelOutput.substr(m.position());
    std::ofstream debug4{std::string{std::filesystem::current_path()} + "/debug4.txt"};

    debug4 << modelOutput << "\n\n";
    debug4 << queryResult;

    std::set<int> stationsVisited{};
    std::vector<int> stationPlan{};


    std::regex queryPart(R"(([0-9]+)([.][0-9])?,([0-9]+))");
    std::regex_search(queryResult, m, queryPart);
    for (auto it = std::sregex_iterator(queryResult.begin(), queryResult.end(), queryPart);
         it != std::sregex_iterator(); it++){
        m = *it;
        debug4 << m[0] << ": ->  "  << m[3] << std::endl;

        int currentPosition = lastLocation; //Needs to be generalized to the current position in the given matrix.

        if(m[1] != "0" && stationsVisited.find(std::stoi(m[3])) == stationsVisited.end()){
            if(stationPlan.empty() && std::stoi(m[3]) == currentPosition) {
                continue;
            }
            stationPlan.push_back(std::stoi(m[3]));
            stationsVisited.insert(std::stoi(m[3]));
        }
    }

    debug4 << "Station plan:\n";
    for(int s : stationPlan)
        debug4 << s << std::endl;


    debug4.close();
    return stationPlan;
}

std::string SingleThreadUppaalBot::runStationModel(){
    std::string verifyta{"/home/martin/Downloads/stratego-bin-cda374c40dd40e9c7b07c2799111d322dcdfc437/verifyta"};
    //std::string verifyta{"./bin-Linux/verifyta"};
    std::string old_model_path{"./" + GetId() + "/station_model.xml"};

    long seed = generateSeed();
    std::string folder_path = "./" + GetId() + "/" + std::to_string(seed);
    std::string new_model_path = folder_path + "/station_model.xml";

    if (mkdir(folder_path.c_str(), 0777) == -1) {
        throw std::runtime_error("Cannot write seed folder");
    }
    std::filesystem::copy(old_model_path, new_model_path);

    std::string runs = std::to_string(num_of_runs);
    store_data("StationPlanSeed", std::to_string(seed));
    std::string terminalCommand = verifyta + " --max-iterations 1 --reset-no-better 1 --good-runs " + runs  + " --total-runs " + runs + " --runs-pr-stat " + runs + " -r " + std::to_string(seed) + " " + new_model_path;

    std::string result;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    stream = popen(terminalCommand.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != nullptr) result.append(buffer);
        pclose(stream);
    }

    std::filesystem::remove(old_model_path);
    print_string(result);

    if(result.find("Failed to learn strategy") != std::string::npos){
        log_helper("Failed to find station strategy");
        store_data("FailedStationStategyGeneration", std::to_string(getLogicalTime()));
        std::cerr << m_strId << " failed to construct Station Strategy" << std::endl;
        throw StrategySynthesisError(m_strId + " failed to construct Station Strategy");
    }

    return result;

}

std::string SingleThreadUppaalBot::runWaypointModel(){
    std::string verifyta{"/home/martin/Downloads/stratego-bin-cda374c40dd40e9c7b07c2799111d322dcdfc437/verifyta"};
    //std::string verifyta{"./bin-Linux/verifyta"};
    std::string old_model_path{"./" + GetId() + "/waypoint_model.xml"};

    long seed = generateSeed();
    std::string folder_path = "./" + GetId() + "/" + std::to_string(seed);
    std::string new_model_path = folder_path + "/waypoint_model.xml";

    if (mkdir(folder_path.c_str(), 0777) == -1) {
        throw std::runtime_error("Cannot write seed folder");
    }
    std::filesystem::copy(old_model_path, new_model_path);

    std::string runs = std::to_string(num_of_runs);
    store_data("WaypointPlanSeed", std::to_string(seed));
    std::string terminalCommand = verifyta + " --max-iterations 1 --reset-no-better 1 --good-runs " + runs  + " --total-runs " + runs + " --runs-pr-stat " + runs + " -r " + std::to_string(seed) + " " + new_model_path;

    std::string result;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    stream = popen(terminalCommand.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != nullptr) result.append(buffer);
        pclose(stream);
    }

    print_string(result);
    std::filesystem::remove(old_model_path);

    if(result.find("Failed to learn strategy") != std::string::npos){
        log_helper("Failed to find waypoint strategy");
        store_data("FailedWaypointStategyGeneration", std::to_string(getLogicalTime()));
        std::cerr << m_strId << " failed to construct Waypoint Strategy" << std::endl;
        throw StrategySynthesisError(m_strId + " failed to construct Waypoint Strategy");
    }

    return result;

}

long SingleThreadUppaalBot::generateSeed() {
    auto now = std::chrono::high_resolution_clock::now();

    return now.time_since_epoch().count();
}

void SingleThreadUppaalBot::constructStationUppaalModel(uint failed){
    if(failed >= 10){
        throw std::runtime_error(m_strId + " have failed Waypoint plan synthesis.");
    }


    std::ifstream partial_blueprint{std::string{std::filesystem::current_path()} + "/planning_blueprint.xml"};
    std::ofstream full_model{"./" + GetId() + "/station_model.xml"};

    // This is the Uppaal model for the initial strategy.
    std::string line;
    int numOfStations;
    std::string matrix;

    if(lastLocation >= sMap.getAmountOfStations()){
        matrix = get_expanded_distance_matrix(sMap, lastLocation);
        numOfStations = sMap.stationIDs.size() + sMap.endStationIDs.size() + 1;
    }
    else {
        matrix = formatMatrix(sMap.floydShortestOfStations());
        numOfStations = sMap.stationIDs.size() + sMap.endStationIDs.size();
    }

    // Replace all the possible "inf" in the matrix.
    size_t index = 0;
    while((index = matrix.find("inf", index)) != std::string::npos){
        matrix.replace(index, 3, "0.0");

        index += 3;
    }

    while(std::getline(partial_blueprint, line)){



        auto pos = line.find("#MAX_STATIONS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#MAX_STATIONS#"}.size(),
                std::to_string(numOfStations));
        }

        pos = line.find("#CUR_STATION#");
        if(pos != std::string::npos){
            //@todo: Make proper functions to encapsulate the number written!
            // The id matches the last index of the expanded DistMatrix.
            if(lastLocation >= sMap.getAmountOfStations()){
                line.replace(pos, std::string{"#CUR_STATION#"}.size(),
                             std::to_string(sMap.getAmountOfStations()));
            }
            else
                line.replace(pos, std::string{"#CUR_STATION#"}.size(),
                    std::to_string(lastLocation));
        }

        pos = line.find("#OTHER_ROBOTS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#OTHER_ROBOTS#"}.size(),
                         std::to_string(numOfOtherActiveRobots(otherBots)));
        }


        pos = line.find("#CUR_ORDER#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#CUR_ORDER#"}.size(),
                         format_order(numOfStations, currentJob->getRemainingStations()));
        }

        pos = line.find("#DISTANCE_MATRIX#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#DISTANCE_MATRIX#"}.size(), matrix);
        }

        pos = line.find("#END_STATIONS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#END_STATIONS#"}.size(),
                format_endstations(numOfStations, currentJob->getEndStations()));
        }

        pos = line.find("#XML_COMMENT_START#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#XML_COMMENT_START#"}.size(), "<!--");
            else
                line.replace(pos, std::string{"#XML_COMMENT_START#"}.size(), "");
        }


        pos = line.find("#XML_COMMENT_END#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#XML_COMMENT_END#"}.size(), "-->");
            else
                line.replace(pos, std::string{"#XML_COMMENT_END#"}.size(), "");
        }

        pos = line.find("#CODE_COMMENT_START#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#CODE_COMMENT_START#"}.size(), "/*");
            else
                line.replace(pos, std::string{"#CODE_COMMENT_START#"}.size(), "");
        }

        pos = line.find("#CODE_COMMENT_END#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#CODE_COMMENT_END#"}.size(), "*/");
            else
                line.replace(pos, std::string{"#CODE_COMMENT_END#"}.size(), "");
        }

        pos = line.find("#REQUIRE_ENDSTATIONS_START#");
        if(pos != std::string::npos){
                line.replace(pos, std::string{"#REQUIRE_ENDSTATIONS_START#"}.size(), "");
        }

        pos = line.find("#REQUIRE_ENDSTATIONS_END#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#REQUIRE_ENDSTATIONS_END#"}.size(), "");
        }

        pos = line.find("#END_AT_ENDSTATION#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#END_AT_ENDSTATION#"}.size(), "end_stations[s]");
        }


        pos = line.find("#OTHER_IN_SYSTEM#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#OTHER_IN_SYSTEM#"}.size(), "");
            else {
                line.replace(pos, std::string{"#OTHER_IN_SYSTEM#"}.size(), "OtherRobot, ");
            }

        }

        pos = line.find("#STATE_VARS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#STATE_VARS#"}.size(),
                         format_state_vars(numOfStations));
        }

        pos = line.find("#POINT_VARS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#STATE_VARS#"}.size(),
                         format_point_vars(numOfStations));
        }

        pos = line.find("#QUERY_TIME#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#QUERY_TIME#"}.size(),
                         std::to_string(5000 * (1 + failed)));
        }

        pos = line.find("#SIMULATION#");
        if(pos != std::string::npos){
            std::string timeframe = std::to_string(5000 * (1 + failed));

            line.replace(pos, std::string{"#QUERY_TIME#"}.size(),
                         timeframe);
        }


        pos = line.find("#GRAPH_BOUND#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#GRAPH_BOUND#"}.size(),
                         "&lt; " + std::to_string(sMap.getAmountOfStations()));
        }

        //********************* Helper functions for when there are other active robots
        if(numOfOtherActiveRobots(otherBots) > 0) {
            pos = line.find("#OTHER_ORDER_LENGHT#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_ORDER_LENGHT#"}.size(),
                             formatStationOrderLenghts(otherBots));
            }

            pos = line.find("#OTHER_START_LOCS#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_START_LOCS#"}.size(),
                             formatOrthersStartLocs(otherBots));
            }

            pos = line.find("#OTHER_PLANS#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_PLANS#"}.size(),
                             formatOtherStationPlan(otherBots, numOfStations * 2));
            }

            pos = line.find("#OTHER_ORDERS#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_ORDERS#"}.size(),
                             formatOtherOrders(otherBots, numOfStations));
            }

            pos = line.find("#OTHER_DISTANCES#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_DISTANCES#"}.size(),
                             formatOtherStationDistances(otherBots, sMap));
            }

            pos = line.find("#OTHER_WORKING#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_WORKING#"}.size(),
                             formatOtherWorking(otherBots));
            }

            pos = line.find("#OTHER_WORKED#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_WORKED#"}.size(),
                             formatWorkedTime(otherBots));
            }
        }

        full_model << line << std::endl;

    }

    full_model.close();
}

void SingleThreadUppaalBot::constructWaypointUppaalModel(uint failed){

    if(failed >= 10){
        throw std::runtime_error(m_strId + " have failed Waypoint plan synthesis.");
    }

    std::ifstream partial_blueprint{std::string{std::filesystem::current_path()} + "/planning_blueprint.xml"};
    std::ofstream waypoint_model{"./" + GetId() + "/waypoint_model.xml"};

    // This is the Uppaal model for the planning blueprint.
    std::string line;

    while(std::getline(partial_blueprint, line)){

        auto pos = line.find("#MAX_STATIONS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#MAX_STATIONS#"}.size(),
                         std::to_string(sMap.points.size())); //@Todo: Have proper getter!
        }

        pos = line.find("#CUR_STATION#");
        if(pos != std::string::npos){
            //@todo: Make proper functions to encapsulate the number written!
            // The id matches the last index of the expanded DistMatrix.
            line.replace(pos, std::string{"#CUR_STATION#"}.size(),
                         std::to_string(lastLocation));
        }

        pos = line.find("#OTHER_ROBOTS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#OTHER_ROBOTS#"}.size(),
                         std::to_string(numOfOtherActiveRobots(otherBots)));
        }


        std::vector<std::vector<float>> matrix = getDistanceMatrix(sMap);

        pos = line.find("#CUR_ORDER#");
        if (pos != std::string::npos) {
            std::set<int> nextStation{};
            nextStation.insert(stationPlan.front());
            line.replace(pos, std::string{"#CUR_ORDER#"}.size(),
                         format_order(matrix.size(), nextStation));
        }

        pos = line.find("#DISTANCE_MATRIX#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#DISTANCE_MATRIX#"}.size(), formatMatrix(matrix));
        }

        pos = line.find("#END_STATIONS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#END_STATIONS#"}.size(),
                         format_endstations(matrix.size(), currentJob->getEndStations()));
        }

        pos = line.find("#XML_COMMENT_START#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#XML_COMMENT_START#"}.size(), "<!--");
            else
                line.replace(pos, std::string{"#XML_COMMENT_START#"}.size(), "");
        }

        pos = line.find("#XML_COMMENT_END#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#XML_COMMENT_END#"}.size(), "-->");
            else
                line.replace(pos, std::string{"#XML_COMMENT_END#"}.size(), "");
        }

        pos = line.find("#CODE_COMMENT_START#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#CODE_COMMENT_START#"}.size(), "/*");
            else
                line.replace(pos, std::string{"#CODE_COMMENT_START#"}.size(), "");
        }

        pos = line.find("#CODE_COMMENT_END#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#CODE_COMMENT_END#"}.size(), "*/");
            else
                line.replace(pos, std::string{"#CODE_COMMENT_END#"}.size(), "");
        }

        pos = line.find("#REQUIRE_ENDSTATIONS_START#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#REQUIRE_ENDSTATIONS_START#"}.size(), "/*");
        }

        pos = line.find("#REQUIRE_ENDSTATIONS_END#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#REQUIRE_ENDSTATIONS_END#"}.size(), "*/");
        }

        pos = line.find("#END_AT_ENDSTATION#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#END_AT_ENDSTATION#"}.size(), "true");
        }

        pos = line.find("#OTHER_IN_SYSTEM#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#OTHER_IN_SYSTEM#"}.size(), "");
            else {
                line.replace(pos, std::string{"#OTHER_IN_SYSTEM#"}.size(), "OtherRobot, ");
            }
        }

        pos = line.find("#STATE_VARS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#STATE_VARS#"}.size(),
                         format_state_vars(sMap.points.size())); //@todo: Make proper getter!
        }

        pos = line.find("#POINT_VARS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#STATE_VARS#"}.size(),
                         format_point_vars(sMap.points.size()));
        }

        pos = line.find("#QUERY_TIME#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#QUERY_TIME#"}.size(),
                         std::to_string(5000 * (1 + failed)));
        }

        pos = line.find("#SIMULATION#");
        if(pos != std::string::npos){
            std::string timeframe = std::to_string(5000 * (1 + failed));

            line.replace(pos, std::string{"#QUERY_TIME#"}.size(),
                         timeframe);
        }

        pos = line.find("#GRAPH_BOUND#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#GRAPH_BOUND#"}.size(),
                         "&gt;= " + std::to_string(sMap.getAmountOfStations() + 1 + numOfOtherRobots(otherBots))); //Must be fixed to the the number of robots on the map, not just the active ones.
        }

        //********************* Helper functions for when there are other active robots
        if(numOfOtherActiveRobots(otherBots) > 0) {
            int numOfStations = sMap.points.size();
            pos = line.find("#OTHER_ORDER_LENGHT#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_ORDER_LENGHT#"}.size(),
                             formatWaypointOrderLenghts(otherBots));
            }

            pos = line.find("#OTHER_START_LOCS#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_START_LOCS#"}.size(),
                             formatOrtherWaypointStartLocs(otherBots));
            }

            pos = line.find("#OTHER_PLANS#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_PLANS#"}.size(),
                             formatOtherWaypointPlan(otherBots, numOfStations * 2));
            }

            pos = line.find("#OTHER_ORDERS#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_ORDERS#"}.size(),
                             formatOtherWaypointOrders(otherBots, numOfStations));
            }

            pos = line.find("#OTHER_DISTANCES#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_DISTANCES#"}.size(),
                             formatOtherWaypointDistances(otherBots, sMap));
            }

            pos = line.find("#OTHER_WORKING#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_WORKING#"}.size(),
                             formatOtherWorking(otherBots));
            }

            pos = line.find("#OTHER_WORKED#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_WORKED#"}.size(),
                             formatWorkedTime(otherBots));
            }
        }

        waypoint_model << line << std::endl;

    }

    waypoint_model.close();
}

REGISTER_CONTROLLER(SingleThreadUppaalBot, "SingleThreadUppaalBot_controller")