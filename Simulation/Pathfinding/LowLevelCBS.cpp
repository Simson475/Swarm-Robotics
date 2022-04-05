#include "LowLevelCBS.hpp"

Path LowLevelCBS::getIndividualPath(std::shared_ptr<Graph> graph, AgentInfo agent, std::vector<Constraint> constraints){
    std::shared_ptr<Vertex> u;
    Action firstAction = agent.getCurrentAction();
    std::shared_ptr<Vertex> goal = agent.getGoal();

    // Compute path from after the current action
    std::priority_queue<ActionPathAux> priorityQueue{};
    priorityQueue.push(ActionPathAux(
        firstAction,
        firstAction.timestamp + firstAction.duration + graph->heuristicCost(firstAction.endVertex, goal),
        nullptr
    ));
    float currentTime = firstAction.timestamp;// The first action is in the prio queue, so its duration will be added later
    this->iterations = 0;
    while( ! priorityQueue.empty()){
        this->iterations++;
        // Next action
        auto top = priorityQueue.top(); priorityQueue.pop();
        u = top.action.endVertex;
        currentTime += top.action.duration;
        if (this->iterations > 25000){
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
                action.timestamp + action.duration + graph->heuristicCost(action.endVertex, goal),
                std::make_shared<ActionPathAux>(top)
            ));
        }
    }
    Error::log("ERROR: No path could be found\n");
    exit(1);
}

std::vector<Path> LowLevelCBS::getAllPaths(std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents, std::vector<Constraint> constraints){
    std::vector<Path> paths{agents.size()};
    int i = 0;
    for (AgentInfo agent : agents){
        paths[i] = getIndividualPath(graph, agent, constraints);
        i++;
    }
    return paths;
}

std::vector<Action> LowLevelCBS::getPossibleActions(std::shared_ptr<Vertex> vertex, AgentInfo agent, std::vector<Constraint> constraints, uint currentTime){
    std::vector<Action> actions; // The actions we will return later (gets filled in)
    std::vector<std::shared_ptr<Edge>> edges = vertex->getEdges();
    float minWaitTime = std::numeric_limits<float>::infinity();
    // Edge actions
    for (auto edge : edges){
        bool edgeIsPossible = true;
        for (Constraint &constraint : constraints){//TODO if this is too slow, we can extract the relevant constraints before the outer loop
            if (constraint.agent.getId() != agent.getId()
            || (constraint.timeEnd < currentTime)
            ){
                continue;//This constraint is irrelevant (not this agent or over before this time)
            }
            // Edge constraints
            if (constraint.location.type == ELocationType::EDGE_LOCATION
             && edge == constraint.location.edge
             && constraint.timeStart < (currentTime + edge->getCost())
            ){
                minWaitTime = (minWaitTime < constraint.timeEnd) ? minWaitTime : constraint.timeEnd;
                edgeIsPossible = false;
                continue;
            }

            // Vertex constraints
            if (constraint.location.type == ELocationType::VERTEX_LOCATION
             && edge->getEndVertex() == constraint.location.vertex//TODO do we need to check the start vertex aswell or will those never reach this point?
             && constraint.timeStart < (currentTime + edge->getCost() + agent.getTimeAtVertex(edge->getEndVertex()))
             && constraint.timeEnd >= (currentTime + edge->getCost() + agent.getTimeAtVertex(edge->getEndVertex()))
            ){
                float arrivalTime = edge->getCost();
                minWaitTime = (minWaitTime < arrivalTime) ? minWaitTime : (constraint.timeEnd - (currentTime + arrivalTime));
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