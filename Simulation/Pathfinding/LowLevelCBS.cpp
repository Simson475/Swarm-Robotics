#include "LowLevelCBS.hpp"

Path LowLevelCBS::getIndividualPath(std::shared_ptr<Graph> graph, std::shared_ptr<Agent> agent, std::vector<Constraint> constraints){
    std::shared_ptr<Vertex> u;
    Action firstAction = agent->getBot()->getCurrentAction();
    std::shared_ptr<Vertex> goal = agent->getGoal();
    // Compute path from after the current action
    std::priority_queue<ActionPathAux, std::vector<ActionPathAux>, std::greater<ActionPathAux>> priorityQueue{};
    priorityQueue.push(ActionPathAux(
        firstAction,
        firstAction.timestamp + std::ceil(firstAction.duration + graph->heuristicCost(firstAction.endVertex, agent->getGoal())),
        nullptr
    ));
    uint currentTime = firstAction.timestamp;
    int iterations = 0;
    while( ! priorityQueue.empty()){
        iterations++;
        // Next action
        auto top = priorityQueue.top(); priorityQueue.pop();
        u = top.action.endVertex;
        
        currentTime += std::ceil(top.action.duration);

        if (iterations > 25000){
            Error::log("Max iterations reached.\n");
            exit(1);
        }
        // If the action leads to the goal we are done
        if (u->getId() == goal->getId()){
            return top.getPath();
        }

        // Expand frontier
        std::vector<Action> possibleActions = getPossibleActions(u, agent, constraints, currentTime);
        for (Action action : possibleActions){
            priorityQueue.push(ActionPathAux(
                action,
                action.timestamp + std::ceil(action.duration + graph->heuristicCost(action.endVertex, agent->getGoal())),
                std::make_shared<ActionPathAux>(top)
            ));
        }
    }
    Error::log("ERROR: No path could be found\n");
    exit(1);
}

std::vector<Path> LowLevelCBS::getAllPaths(std::shared_ptr<Graph> graph, std::vector<std::shared_ptr<Agent>> agents, std::vector<Constraint> constraints){
    std::vector<Path> paths{agents.size()};
    int i = 0;
    for (std::shared_ptr<Agent> agent : agents){
        paths[i] = getIndividualPath(graph, agent, constraints);
        i++;
    }
    return paths;
}

std::vector<Action> LowLevelCBS::getPossibleActions(std::shared_ptr<Vertex> vertex, std::shared_ptr<Agent> agent, std::vector<Constraint> constraints, uint currentTime){
    std::vector<Action> actions;
    std::vector<std::shared_ptr<Edge>> edges = vertex->getEdges();
    uint minWaitTime = -1;//Max uint value
    // Edge actions
    for (auto edge : edges){
        bool edgeIsPossible = true;
        for (Constraint &constraint : constraints){//TODO if this is too slow, we can extract the relevant constraints before the outer loop
            if (constraint.agent->getId() != agent->getId()
            || (constraint.timeEnd < currentTime)
            ){
                continue;//This constraint is irrelevant (not this agent or over before this time)
            }
            // Edge constraints
            if (constraint.location.type == ELocationType::EDGE_LOCATION
             && edge == constraint.location.edge
             && constraint.timeStart < (currentTime + std::ceil(edge->getCost()))
            ){
                minWaitTime = (minWaitTime < constraint.timeEnd) ? minWaitTime : constraint.timeEnd;
                edgeIsPossible = false;
                continue;
            }

            // Vertex constraints
            if (constraint.location.type == ELocationType::VERTEX_LOCATION
             && edge->getEndVertex() == constraint.location.vertex//TODO do we need to check the start vertex aswell or will those never reach this point?
             && constraint.timeStart < (currentTime + agent->getTimeAtVertex(edge->getEndVertex()))
            ){
                minWaitTime = (minWaitTime < constraint.timeEnd) ? minWaitTime : constraint.timeEnd;
                edgeIsPossible = false;
                continue;
            }
        }

        if (edgeIsPossible){
            actions.push_back(Action(
                currentTime,
                edge->getStartVertex(),
                edge->getEndVertex(),
                edge->getCost()
            ));
        }
    }

    if (actions.size() == edges.size()){
        // We have added all possible edges, no need to wait
        return actions;
    }

    // Wait action
    actions.push_back(Action(currentTime, vertex, vertex, minWaitTime));

    return actions;
}

// Path LowLevelCBS::constructPathFromPlan(Graph *graph, std::vector<Vertex> plan){
//     if (plan.size() < 2) { return Path{}; }// Prevent nasty things
    
//     std::vector<Action*> actions{};
//     float startTime = 0;
//     for (std::vector<Point>::size_type i=0; i < plan.size()-2; i++){
//         Action* action = new Action();
//         action->startVertex = plan[i];
//         action->endVertex = plan[i+1];
//         action->timestamp = startTime;
//         float cost = 0;
//         for(std::shared_ptr<Edge> e : plan[i].getEdges()){
//             if (e->getEndVertex().getId() == plan[i+1].getId()){
//                 cost = e->getCost();
//                 break;
//             }
//         }
//         action->cost = cost;//WARNING cost changed
//         startTime += cost;
//         actions.push_back(action);
//     }
    
//     return Path{actions: actions, cost: startTime };
// }

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