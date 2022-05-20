#ifndef HIGHLEVEL_CBS_HPP
#define HIGHLEVEL_CBS_HPP

class HighLevelCBS;

#include "ConstraintTree.hpp"
#include "Constraint.hpp"
#include <vector>
#include <queue>
#include <memory>
#include <deque>
#include <array>
#include "Conflict.hpp"
#include "Solution.hpp"
#include "Graph.hpp"
#include "AgentInfo.hpp"
#include "LowLevelCBS.hpp"
#include "Debugging.hpp"
#include "Logger.hpp"
#include <chrono>
#include "DEFINITIONS.hpp"
#include <iostream>

// This must be AFTER "DEFINITIONS.hpp" has been included
#ifdef ARGOS
#include "argos3/core/control_interface/ci_controller.h"
#include "argos3/core/simulator/loop_functions.h"
#endif

class HighLevelCBS {    
public:
    // Singleton
    static HighLevelCBS &get_instance() {
        static HighLevelCBS instance;
        #ifdef ARGOS
        try {
            argos::TConfigurationNode &t_node = argos::CSimulator::GetInstance().GetConfigurationRoot();
            argos::TConfigurationNode &params = argos::GetNode(t_node, "experiment_settings");
            argos::GetNodeAttribute(params, "timeout", instance.timeout);
        }
        catch (argos::CARGoSException &e){
            std::cerr << "High level timeout defaulted to: " << instance.timeout << std::endl;
        }
        #endif
        return instance;
    }
    Solution findSolution(std::shared_ptr<Graph>, std::vector<AgentInfo>, LowLevelCBS&, int maxTime = -1);
    Conflict getBestConflict(std::shared_ptr<ConstraintTree>, std::shared_ptr<Graph>, std::vector<AgentInfo>, std::vector<Conflict>, LowLevelCBS&);
    uint iterations;
private:
    void blockGoalsForever(Solution& solution);
    void removeInfiniteBlocksOnGoals(Solution& solution);
    Solution getGreedySolution(std::shared_ptr<Graph>, std::vector<AgentInfo>, LowLevelCBS&);
    int timeout = 100000; //0.1 seconds
};

#endif