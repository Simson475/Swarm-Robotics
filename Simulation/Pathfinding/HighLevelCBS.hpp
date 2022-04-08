#ifndef HIGHLEVEL_CBS_HPP
#define HIGHLEVEL_CBS_HPP

class HighLevelCBS;

#include "argos_wrapper/argos_wrapper.hpp"
#include "map_structure.hpp"
#include "ConstraintTree.hpp"
#include "Constraint.hpp"
//#include "SingleThreadBotCBS.hpp"
#include <vector>
#include <queue>
#include <memory>
#include <deque>
#include <array>
#include "Conflict.hpp"
#include "Solution.hpp"
#include "TestController.hpp"
#include "Graph.hpp"
#include "AgentInfo.hpp"
#include "LowLevelCBS.hpp"
#include "ExperimentData.hpp"

#include "Debugging.hpp"
#include "Logger.hpp"

class HighLevelCBS {    
public:
    // Singleton
    static HighLevelCBS &get_instance() {
        static HighLevelCBS instance;
        return instance;
    }
    Solution findSolution(std::shared_ptr<Graph>, std::vector<AgentInfo>, LowLevelCBS);
    Conflict getBestConflict(std::shared_ptr<ConstraintTree>, std::shared_ptr<Graph>, std::vector<AgentInfo>, std::vector<Conflict>, LowLevelCBS);
};

#endif