#include "LowLevelCBS.hpp"

/**
 * Current problems
 * - Constraints probably need to be sorted in order of starting time
 * - The current time is needed when calculating the plans unless the current action starts at the exact time we are at
 *   - Should be simplest by modifying timestamp to the current time for all actions before calling highlevel?
 * - If we split goal actions during second plan generation we need to fix it in the controllers so they leave working state to follow plan
 *   - 
 * - We hit a blocked goal problem, that needs fixing
 */
/**
 * Current room for improvement
 * - Actions etc can most likely be references and speed things up
 */

/**
 * PRE: all the constraints must be for the agent
 * 
 * @param graph 
 * @param agent 
 * @param constraints 
 * @return Path 
 */
Path LowLevelCBS::getIndividualPath(std::shared_ptr<Graph> graph, AgentInfo agent, std::vector<Constraint> constraints, float currentTime){
    #ifdef DEBUG_LOGS_ON
    Error::log("A constraints for agent: \n");
    for (auto constr : constraints){
        Error::log(constr.toString() + "\n");
    }
    #endif
    #ifdef LOWLEVEL_ANALYSIS_LOGS_ON
    Logger& logger = Logger::get_instance();
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    #endif

    std::shared_ptr<Vertex> u;
    Action firstAction = agent.getCurrentAction();
    std::shared_ptr<Vertex> goal = agent.getGoal();

    // If the initial action is a wait action we can reduce the wait time if there is a constraint on it
    if (firstAction.isWaitAction() && ConstraintUtils::isViolatingConstraint(constraints, firstAction)){
        Constraint constraint = ConstraintUtils::getViolatedConstraint(constraints, firstAction);
        float timeUntilConstraint = constraint.timeStart - currentTime;
        if (timeUntilConstraint <= TIME_AT_VERTEX){
            throw std::string("No path could be found\n");
        }
        firstAction.duration = timeUntilConstraint - TIME_AT_VERTEX;
    }
    
    // If the first action is the goal action (special case if duration == 0, since it means the agent is done and needs no path)
    if (firstAction.isWaitAction() && firstAction.endVertex == goal && (firstAction.duration == TIME_AT_GOAL || firstAction.duration == 0)){
        return {{firstAction}, firstAction.timestamp + firstAction.duration};
    }

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

        if (this->iterations > 5000000){
            this->totalIterations += this->iterations;
            Error::log("Max iterations reached.\n");
            exit(1);
        }
        if (top.action.endVertex == goal && canWorkAtGoalWithoutViolatingConstraints(top.action, goal, constraints)){
            this->totalIterations += this->iterations;
            #ifdef LOWLEVEL_ANALYSIS_LOGS_ON
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            auto timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            (*logger.begin()) << this->iterations << " iterations took " << timeDiff << "[µs] for low level individual path\n"; logger.end();
            #endif
            // Return the path, we have found a path that violates no constraints.
            return top.getPath();
        }

        // Expand frontier
        std::vector<Action> possibleActions = getPossibleActions(u, constraints, top.action.timestamp + top.action.duration);
        for (Action action : possibleActions){
            // Check if we have already visited the vertex (at the given time) the action is taking us to
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
            // bool visitedAtAnyTime = false;
            // for (Action a : top.getPath().actions){
            //     if (a.startVertex == action.endVertex || a.endVertex == action.endVertex){
            //         visitedAtAnyTime = true;
            //         break;
            //     }
            // }
            // bool hasConstraint = false;
            // for (Constraint &constraint : constraints){
            //     // If there is a constraint for the agent at the actions end vertex
            //     if (constraint.location.type == ELocationType::VERTEX_LOCATION
            //      && constraint.location.vertex == action.endVertex
            //     ){
            //         hasConstraint = true;
            //         break;
            //     }
            // }
            // if (visitedAtAnyTime && ! hasConstraint && ! action.isWaitAction()) continue;
            
            // Add the vertex to the visited vertices
            vertexVisits[action.endVertex->getId()].push_back(action.timestamp + action.duration);

            auto aux = ActionPathAux(
                action,
                graph->heuristicCost(action.endVertex, goal),
                std::make_shared<ActionPathAux>(top)
            );

            priorityQueue.push(aux);
        }
    }
    throw std::string("No path could be found\n");
}

std::vector<Path> LowLevelCBS::getAllPaths(std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents, std::vector<std::vector<Constraint>> constraints, float currentTime){
    std::vector<Path> paths{agents.size()};
    int i = 0;
    for (AgentInfo agent : agents){
        paths[i] = getIndividualPath(graph, agent, constraints[agent.getId()], currentTime);
        i++;
    }
    return paths;
}

std::vector<Action> LowLevelCBS::getPossibleActions(std::shared_ptr<Vertex> vertex, std::vector<Constraint> constraints, float currentTime){
    std::vector<Action> actions; // The actions we will return later (gets filled in)
    std::vector<std::shared_ptr<Edge>> edges = vertex->getEdges();
    float minWaitTime = std::numeric_limits<float>::infinity();
    // Edge actions
    for (auto edge : edges){
        bool edgeIsPossible = true;
        for (Constraint &constraint : constraints){
            // Edge constraints
            if (ConstraintUtils::isViolatingConstraint(constraint, edge, currentTime)){
                minWaitTime = std::min(minWaitTime, constraint.timeEnd + DELTA);
                edgeIsPossible = false;
                continue;
            }

            // Vertex constraints
            if (ConstraintUtils::isViolatingConstraint(constraint, edge->getEndVertex(), currentTime + edge->getCost(), currentTime + edge->getCost() + TIME_AT_VERTEX)){
                minWaitTime = std::min(minWaitTime, constraint.timeEnd - (currentTime + edge->getCost()) + DELTA);
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
    for (Constraint &constraint : constraints){
        if (ConstraintUtils::isViolatingConstraint(constraint, vertex, currentTime + minWaitTime, currentTime + minWaitTime + TIME_AT_VERTEX)){
            return actions;// We cant wait here to take the edge that set min wait time, so no point in waiting here.
        }
    }
    actions.push_back(Action(currentTime, vertex, vertex, minWaitTime));

    return actions;
}

/**
 * PRE: action ends at goal
 * 
 * @param action
 * @param goal 
 * @param constraints 
 * @return true 
 * @return false 
 */
bool LowLevelCBS::canWorkAtGoalWithoutViolatingConstraints(Action action, std::shared_ptr<Vertex> goal, std::vector<Constraint> constraints){
    float arrivalTime = action.timestamp + action.duration;
    for (auto c : constraints){
        if (ConstraintUtils::isViolatingConstraint(c, goal, arrivalTime, arrivalTime + TIME_AT_GOAL + TIME_AT_VERTEX)){
            return false;
        }
    }
    return true;
}