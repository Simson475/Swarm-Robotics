#include "LowLevelCBS.hpp"

Path LowLevelCBS::getIndividualPath(std::shared_ptr<Graph> graph, AgentInfo agent, std::vector<Constraint> constraints){
    std::shared_ptr<Vertex> u;
    Action firstAction = agent.getCurrentAction();
    std::shared_ptr<Vertex> goal = agent.getGoal();

    // Compute path from after the current action
    std::priority_queue<ActionPathAux, std::vector<ActionPathAux>, std::greater<ActionPathAux>> priorityQueue{};
    // Since we can legally visit each vertex multiple times at different time steps
    // we will need more than a regular visited list/set. To run faster, we will
    // have a list for each vertex, that contains the timesteps that we have visited the vertex.
    std::vector<std::vector<float>> vertexVisits{graph->getVertices().size()};
    priorityQueue.push(ActionPathAux(
        firstAction,
        firstAction.timestamp + firstAction.duration + graph->heuristicCost(firstAction.endVertex, goal),
        nullptr
    ));
    Logger logger = Logger("LowLevel.txt", false);
    this->iterations = 0;
    while( ! priorityQueue.empty()){
        this->iterations++;
        // Next action
        auto top = priorityQueue.top(); priorityQueue.pop();
        u = top.action.endVertex;
        // std::cout << "Top action: " << top.action.toString() << "\n";

        if (this->iterations > 5000){
            Error::log("Max iterations reached.\n");
            exit(1);
        }
        // If the action leads to the goal we are done only if we can wait there for the time we need at the goal
        if (u->getId() == goal->getId()){
            float currentTime = top.action.timestamp + top.action.duration;
            bool canSpendNeededTimeAtGoal = true;
            for (Constraint &constraint : constraints){//TODO if this is too slow, we can extract the relevant constraints before the outer loop
                if (constraint.agentId != agent.getId()
                || (constraint.timeEnd <= currentTime)
                || (constraint.location.type == ELocationType::EDGE_LOCATION)
                ){
                    continue;//This constraint is irrelevant (not this agent or over before this time)
                }
                // At this point we know constraint.timeEnd > currentTime
                // We want to check if the vertex constraint applies to the goal within the time the agent will be at the goal
                if (constraint.timeStart < currentTime + TIME_AT_GOAL
                 && constraint.location.vertex == goal){
                    canSpendNeededTimeAtGoal = false;
                    // std::cout << constraint.toString() << "\n" << top.action.toString() << "\n";
                    // exit(1);
                    break;
                }
            }
            if (canSpendNeededTimeAtGoal){
                (*logger.begin()) << iterations << "\n"; logger.end();
                return top.getPath();
            }
        }

        // Expand frontier
        std::vector<Action> possibleActions = getPossibleActions(u, agent, constraints, top.action.timestamp + top.action.duration);
        for (Action action : possibleActions){
            // Check if we have already visited the vertex the action is taking us to
            // Checking end vertices will cover both wait and edge actions.
            bool visited = false;
            for (float t : vertexVisits[action.endVertex->getId()]){
                if (t == action.timestamp + action.duration){
                    // We have already visited that vertex at the given time
                    visited = true;
                    break;
                }
            }
            if (visited) continue;
            //TODO We can probably also restrict cycles to only be allowed iff there is a constraint at some point on the vertex you are cycling to
            // bool visitedAtAnyTime = ! vertexVisits[action.endVertex->getId()].empty();
            // bool hasConstraint = false;
            // for (Constraint &constraint : constraints){//TODO if this is too slow, we can extract the relevant constraints before
            //     // If there is a constraint for the agent at the actions end vertex
            //     if (constraint.agentId == agent.getId()
            //      && constraint.location.type == ELocationType::VERTEX_LOCATION
            //      && constraint.location.vertex == action.endVertex
            //     ){
            //         hasConstraint = true;
            //         break;
            //     }
            // }
            // std::cout << visitedAtAnyTime << (! hasConstraint) << (visitedAtAnyTime && (! hasConstraint)) << "\n";
            // if (visitedAtAnyTime && ! hasConstraint) continue;
            
            vertexVisits[action.endVertex->getId()].push_back(action.timestamp + action.duration);

            auto aux = ActionPathAux(
                action,
                graph->heuristicCost(action.endVertex, goal),
                std::make_shared<ActionPathAux>(top)
            );
            // std::cout << "aux: " << aux.toString() << "\n";
            priorityQueue.push(aux);
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
            if (constraint.agentId != agent.getId()
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
             && constraint.timeStart < (currentTime + edge->getCost())
             && constraint.timeEnd >= (currentTime + edge->getCost() + DELTA)
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