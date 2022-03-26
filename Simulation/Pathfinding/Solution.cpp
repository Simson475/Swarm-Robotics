#include "Solution.hpp"

void Solution::setPath(std::shared_ptr<Agent> agent, Path path){
    paths[agent->getId()] = path;
}