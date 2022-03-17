#ifndef LOWLEVEL_CBS_HPP
#define LOWLEVEL_CBS_HPP

#include <vector>
#include "_path.hpp"
#include "AgentInfo.hpp"

class LowLevelCBS {
public:
    // Singleton
    static LowLevelCBS &get_instance() {
        static LowLevelCBS instance;
        return instance;
    }
    Path getIndividualPath(AgentInfo);
    std::vector<Path> getAllPaths(std::vector<AgentInfo>);
};

#endif