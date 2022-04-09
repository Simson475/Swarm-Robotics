#include "LowLevelCBS.hpp"

/**
 * PRE: all the constraints must be for the agent
 * 
 * @param graph 
 * @param agent 
 * @param constraints 
 * @return Path 
 */
Path LowLevelCBS::getIndividualPath(std::shared_ptr<Graph> graph, AgentInfo agent, std::vector<Constraint> constraints){
    Logger logger = Logger::get_instance();
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Error::log("Agent" + std::to_string(agent.getId()) + "\n");

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
    this->iterations = 0;
    while( ! priorityQueue.empty()){
        this->iterations++;
        // Next action
        auto top = priorityQueue.top(); priorityQueue.pop();
        u = top.action.endVertex;
        // Error::log("Top action: " + top.action.toString() + "\n");

        if (this->iterations > 5000){
            this->totalIterations += this->iterations;
            Error::log("Max iterations reached.\n");
            exit(1);
        }
        // If the action leads to the goal we are done only if we can wait there for the time we need at the goal
        if (u->getId() == goal->getId()){
            float currentTime = top.action.timestamp + top.action.duration;
            bool canSpendNeededTimeAtGoal = true;
            for (Constraint &constraint : constraints){
                if (constraint.location.type == ELocationType::EDGE_LOCATION){
                    continue;//This constraint is irrelevant
                }
                // We want to check if the vertex constraint applies to the goal within the time the agent will be at the goal
                if (isViolatingConstraint(constraint, goal, currentTime, currentTime + TIME_AT_GOAL)){
                    canSpendNeededTimeAtGoal = false;
                    break;
                }
            }
            if (canSpendNeededTimeAtGoal){
                if (Logger::enabled) {
                    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                    auto timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
                    (*logger.begin()) << iterations << " iterations took " << timeDiff << "[µs] for low level individual path\n"; logger.end();
                }
                this->totalIterations += this->iterations;
                // Return the path, we have found a path that violates no constraints.
                // Error::log(top.getPath().toString() + "\n");
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
            
            // We can probably also restrict cycles to only be allowed iff there is a constraint at some point on the vertex you are cycling to
            bool visitedAtAnyTime = false;
            for (Action a : top.getPath().actions){
                if (a.startVertex == action.endVertex || a.endVertex == action.endVertex){
                    visitedAtAnyTime = true;
                    break;
                }
            }
            bool hasConstraint = false;
            for (Constraint &constraint : constraints){//TODO if this is too slow, we can extract the relevant constraints before
                // If there is a constraint for the agent at the actions end vertex
                if (constraint.location.type == ELocationType::VERTEX_LOCATION
                 && constraint.location.vertex == action.endVertex
                ){
                    hasConstraint = true;
                    break;
                }
            }
            if (visitedAtAnyTime && ! hasConstraint && ! action.isWaitAction()) continue;
            
            // Add the vertex to the visited vertices
            vertexVisits[action.endVertex->getId()].push_back(action.timestamp + action.duration);

            auto aux = ActionPathAux(
                action,
                graph->heuristicCost(action.endVertex, goal),
                std::make_shared<ActionPathAux>(top)
            );
            // Error::log("aux: " + aux.toString() + "\n");

            // TODO we need something like the following, but it only works for 2 agents. For >2 agents, the problem will persist one step further out
            // // If this action is a wait action for the goal, speed up by not trying to find another path to it (TODO it should not make sense to do so?)
            // if (action.isWaitAction() && action.endVertex->getEdge(goal) != nullptr){
            //     priorityQueue = {};// Clear the priority queue
            //     priorityQueue.push(aux);
            //     break;
            // }

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

std::vector<Action> LowLevelCBS::getPossibleActions(std::shared_ptr<Vertex> vertex, AgentInfo agent, std::vector<Constraint> constraints, float currentTime){
    std::vector<Action> actions; // The actions we will return later (gets filled in)
    std::vector<std::shared_ptr<Edge>> edges = vertex->getEdges();
    float minWaitTime = std::numeric_limits<float>::infinity();
    float delta = 1;
    // Edge actions
    for (auto edge : edges){
        bool edgeIsPossible = true;
        for (Constraint &constraint : constraints){//TODO if this is too slow, we can extract the relevant constraints before the outer loop
            // Edge constraints
            if (isViolatingConstraint(constraint, edge, currentTime)){
                minWaitTime = std::min(minWaitTime, constraint.timeEnd + delta);
                edgeIsPossible = false;
                continue;
            }

            // Vertex constraints
            if (isViolatingConstraint(constraint, edge->getEndVertex(), currentTime + edge->getCost(), currentTime + edge->getCost())){
                minWaitTime = std::min(minWaitTime, constraint.timeEnd - (currentTime + edge->getCost()) + delta);
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

/**
 * PRE: The constraint and action are for the same agent
 * 
 * @param constraint 
 * @param action 
 * @return true
 * @return false 
 */
bool LowLevelCBS::isViolatingConstraint(Constraint constraint, Action action){
    if (constraint.location.type == ELocationType::EDGE_LOCATION){
        if (action.isWaitAction()) return false;
        auto edge = action.startVertex->getEdge(action.endVertex);
        return isViolatingConstraint(constraint, edge, action.timestamp);
    }
    return isViolatingConstraint(constraint, action.endVertex, action.timestamp + action.duration, action.timestamp + action.duration);
}

/**
 * PRE: The constraint and action are for the same agent
 * 
 * @param constraint 
 * @param action 
 * @return true
 * @return false 
 */
bool LowLevelCBS::isViolatingConstraint(Constraint constraint, std::shared_ptr<Edge> edge, float startTime){
    // Within constraint timespan
    float maxStart = std::max(constraint.timeStart, startTime);
    float minEnd = std::min(constraint.timeEnd, startTime + edge->getCost());
    bool withinConstraintsTimespan = maxStart <= minEnd;
    if (withinConstraintsTimespan == false) return false;

    // Violating constraint location
    return (constraint.location.type == ELocationType::EDGE_LOCATION
         && edge == constraint.location.edge);
}

/**
 * PRE: The constraint and action are for the same agent
 * 
 * @param constraint 
 * @param action 
 * @return true
 * @return false 
 */
bool LowLevelCBS::isViolatingConstraint(Constraint constraint, std::shared_ptr<Vertex> vertex, float startTime, float endTime){
    // Within constraint timespan
    float maxStart = std::max(constraint.timeStart, startTime);
    float minEnd = std::min(constraint.timeEnd, endTime);
    bool withinConstraintsTimespan = maxStart <= minEnd;
    if (withinConstraintsTimespan == false) return false;

    // Violating constraint location
    return (constraint.location.type == ELocationType::VERTEX_LOCATION
         && vertex == constraint.location.vertex);
}