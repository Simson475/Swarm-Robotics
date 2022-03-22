#include "LowLevelCBS.hpp"

Path LowLevelCBS::getIndividualPath(Graph *graph, const AgentInfo&){
    Path path;
    return path;
}

std::vector<Path> LowLevelCBS::getAllPaths(Graph *graph, std::vector<AgentInfo> agentInfo){
    std::vector<Path> paths{agentInfo.size()};
    for(auto &info : agentInfo){
        Error::log("_1");
        // Get agents location
        std::shared_ptr<Location> location = info.getLocation();
        Vertex nextVertex;
        Error::log("_2");
        // If it is an edge, we will calculate its path from the next vertex.
        if (location->type == ELocationType::EDGE_LOCATION) {
            nextVertex = location->location.edge.getEndVertex();
        }
        // If it is not an edge, we simply take the vertex
        else{
            nextVertex = location->location.vertex;
        }
        Error::log("_3");

        auto plan = graph->findPath(nextVertex.getId(), info.getGoal()->getId());
        Error::log("_4");
        // Graph path does not include starting point, so we insert it
        plan.insert(plan.begin(), nextVertex);
        Path path = constructPathFromPlan(graph, plan);
        paths[info.getId()] = path;
    }
    return paths;
}

Path LowLevelCBS::constructPathFromPlan(Graph *graph, std::vector<Vertex> plan){
    if (plan.size() < 2) { return Path{}; }// Prevent nasty things
    
    std::vector<Action*> actions{};
    float startTime = 0;
    for (std::vector<Point>::size_type i=0; i < plan.size()-2; i++){
        Action* action = new Action();
        action->startVertex = plan[i];
        action->endVertex = plan[i+1];
        action->timestamp = startTime;
        float cost = 0;
        for(std::shared_ptr<Edge> e : plan[i].getEdges()){
            if (e->getEndVertex().getId() == plan[i+1].getId()){
                cost = e->getCost();
                break;
            }
        }
        action->cost = cost;
        startTime += cost;
        actions.push_back(action);
    }
    
    return Path{actions: actions, cost: startTime };
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