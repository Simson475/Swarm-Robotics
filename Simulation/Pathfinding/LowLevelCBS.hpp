#ifndef LOWLEVEL_CBS_HPP
#define LOWLEVEL_CBS_HPP

class LowLevelCBS;

#include <vector>
#include <memory>
#include "Path.hpp"
#include "AgentInfo.hpp"
#include "Action.hpp"
#include "Debugging.hpp"
#include <queue>
#include "Location.hpp"
#include "Constraint.hpp"
#include "ActionPathAux.hpp"
#include "Vertex.hpp"
#include "Graph.hpp"
#include "ConstraintTree.hpp"
#include "Logger.hpp"
#include "GLOBALS.hpp"
#include <chrono>
#include "DEFINITIONS.hpp"
#include "ConstraintUtils.hpp"

class LowLevelCBS {
public:
    // Singleton
    static LowLevelCBS &get_instance() {
        static LowLevelCBS instance;
        return instance;
    }
    Path getIndividualPath(std::shared_ptr<Graph> graph, AgentInfo agent, std::vector<Constraint> constraints);
    std::vector<Path> getAllPaths(std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents, std::vector<std::vector<Constraint>> constraints);
    uint iterations = 0;
    unsigned long int totalIterations = 0;
protected:
    std::vector<Action> getPossibleActions(std::shared_ptr<Vertex> vertex, std::vector<Constraint> constraints, float currentTime);
    bool canWorkAtGoalWithoutViolatingConstraints(Action action, std::shared_ptr<Vertex> goal, std::vector<Constraint> constraints);
};

#endif