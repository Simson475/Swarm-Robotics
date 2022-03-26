#ifndef LOWLEVEL_CBS_HPP
#define LOWLEVEL_CBS_HPP

class LowLevelCBS;

#include <vector>
#include <memory>
#include "_path.hpp"
#include "Agent.hpp"
#include "Action.hpp"
#include "Debugging.hpp"
#include <queue>
#include "Location.hpp"
#include "Constraint.hpp"
#include "ActionPathAux.hpp"

class LowLevelCBS {
public:
    // Singleton
    static LowLevelCBS &get_instance() {
        static LowLevelCBS instance;
        return instance;
    }
    Path getIndividualPath(std::shared_ptr<Graph> graph, std::shared_ptr<Agent>, std::vector<Constraint> constraints);
    std::vector<Path> getAllPaths(std::shared_ptr<Graph> graph, std::vector<std::shared_ptr<Agent>>, std::vector<Constraint> constraints);
    //Path constructPathFromPlan(std::shared_ptr<Graph> graph, std::vector<Vertex> plan);
protected:
    std::vector<Action> getPossibleActions(std::shared_ptr<Vertex> vertex, std::shared_ptr<Agent> agent, std::vector<Constraint> constraints, uint currentTime);
};

#endif