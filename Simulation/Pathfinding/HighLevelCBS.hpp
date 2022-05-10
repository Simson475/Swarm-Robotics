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

class HighLevelCBS {    
public:
    // Singleton
    static HighLevelCBS &get_instance() {
        static HighLevelCBS instance;
        return instance;
    }
    Solution findSolution(std::shared_ptr<Graph>, std::vector<AgentInfo>, LowLevelCBS&, float currentTime = 0);
    Conflict getBestConflict(std::shared_ptr<ConstraintTree>, std::shared_ptr<Graph>, std::vector<AgentInfo>, std::vector<Conflict>, LowLevelCBS&);
    uint iterations;
private:
    void blockGoalsForever(Solution& solution);
    void removeInfiniteBlocksOnGoals(Solution& solution);
    Solution getGreedySolution(std::shared_ptr<Graph>, std::vector<AgentInfo>, LowLevelCBS&, float currentTime = 0);
};

#endif