#include "LowLevelCBS.hpp"

Path LowLevelCBS::getIndividualPath(const AgentInfo&){
    Path path;
    return path;
}

std::vector<Path> LowLevelCBS::getAllPaths(std::vector<AgentInfo>){
    std::vector<Path> paths;
    return paths;
}


// Solution* solution = new Solution();
// Map_Structure map = Map_Structure::get_instance();
// std::vector<Agent*> allAgents;
// auto botList = getControllers();
// for(TestController* bot : botList){
//     Agent* agent = new Agent();
//     agent->setBot(bot);
//     auto plan = bot->findOptimalPath();
//     agent->createPath(plan);
//     allAgents.push_back(agent);
// };
// solution->agents = allAgents;
// return solution;