#ifndef LOWLEVEL_CBS_HPP
#define LOWLEVEL_CBS_HPP

#include <vector>
#include <memory>
#include "_path.hpp"
#include "AgentInfo.hpp"

#include "Debugging.hpp"

class LowLevelCBS {
public:
    // Singleton
    static LowLevelCBS &get_instance() {
        static LowLevelCBS instance;
        return instance;
    }
    Path getIndividualPath(Graph *graph, const AgentInfo&);
    std::vector<Path> getAllPaths(Graph *graph, std::vector<AgentInfo>);
    Path constructPathFromPlan(Graph *graph, std::vector<Vertex> plan);
};

#endif