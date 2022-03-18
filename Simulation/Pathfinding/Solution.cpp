#include "Solution.hpp"

void Solution::setPath(AgentInfo& agent, Path path){
    paths[agent.getId()] = path;
}