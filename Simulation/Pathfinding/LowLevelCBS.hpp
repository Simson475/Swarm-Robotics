#ifndef LOWLEVEL_CBS_HPP
#define LOWLEVEL_CBS_HPP

class LowLevelCBS;

#include <vector>
#include <memory>
#include "_path.hpp"
#include "AgentInfo.hpp"
#include "Action.hpp"
#include "Debugging.hpp"
#include <queue>
#include "Location.hpp"
#include "Constraint.hpp"
#include "ActionPathAux.hpp"
#include "BackwardsActionPathAux.hpp"
#include "Vertex.hpp"
#include "Graph.hpp"
#include "ConstraintTree.hpp"
#include "Logger.hpp"
#include "GLOBALS.hpp"
#include <chrono>
#include "DEFINITIONS.hpp"

class LowLevelCBS {
public:
    // Singleton
    static LowLevelCBS &get_instance() {
        static LowLevelCBS instance;
        return instance;
    }
    Path getIndividualPath(std::shared_ptr<Graph> graph, AgentInfo agent, std::vector<Constraint> constraints);
    std::vector<Path> getAllPaths(std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents, std::vector<Constraint> constraints);
    uint iterations = 0;
    unsigned long int totalIterations = 0;
protected:
    std::vector<Action> getPossibleActions(std::shared_ptr<Vertex> vertex, std::vector<Constraint> constraints, float currentTime);
    bool isViolatingConstraint(Constraint constraint, Action action);
    bool isViolatingConstraint(Constraint constraint, std::shared_ptr<Edge> edge, float startTime);
    bool isViolatingConstraint(Constraint constraint, std::shared_ptr<Vertex> vertex, float startTime, float endTime);
    bool endsAtValidGoal(Action action, std::shared_ptr<Vertex> goal, std::vector<Constraint> constraints);
};

#endif