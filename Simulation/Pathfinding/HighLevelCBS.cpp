#include "HighLevelCBS.hpp"

Solution HighLevelCBS::findSolution(std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents, LowLevelCBS& lowLevel, float currentTime){
    #ifdef HIGHLEVEL_ANALYSIS_LOGS_ON
    Logger& logger = Logger::get_instance();
    #endif
    #ifdef DEBUG_LOGS_ON
    for (auto a : agents){
        Error::log("Agent" + std::to_string(a.getId()) + ": " + agents[a.getId()].getCurrentAction().getLocation().toString() + " --> " + a.getGoal()->toString() + "\n");
    }
    #endif
    #ifndef EXPERIMENT
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    #endif
    /**
     * Root.constraints = {}
     * Root.solution = find individual paths by the low level
     * Root.cost = SIC(Root.solution)
     */
    std::shared_ptr<ConstraintTree> root = std::make_shared<ConstraintTree>(agents.size());
    #ifndef EXPERIMENT
    // Check if we have conflicts on the initial actions (Means we desynced and wont be able to find a CBS solution)
    Solution currentSolution;
    currentSolution.paths.resize(agents.size());
    for(auto& a : agents){
        currentSolution.paths[a.getId()] = {a.getCurrentAction()};
    }
    root->setSolution(currentSolution);
    auto conflictsOnCurrentActions = root->findConflicts();
    if (conflictsOnCurrentActions.size() > 0){
        return getGreedySolution(graph, agents, lowLevel, currentTime);
    }
    #endif

    // Set initial constraints to avoid conflicts on initial actions
    for (auto a : agents){
        for (auto b : agents){
            if (a.getId() != b.getId()){
                auto initialAction = a.getCurrentAction();
                if (initialAction.isWaitAction()){
                    // Constraint the initial vertex
                    float earliestActionEnd = initialAction.timestamp + initialAction.duration;//(initialAction.duration == TIME_AT_GOAL && initialAction.endVertex == a.getGoal())
                     //? initialAction.timestamp + initialAction.duration : currentTime;
                    root->addConstraint(Constraint(b.getId(), initialAction.getLocation(), currentTime, earliestActionEnd + TIME_AT_VERTEX));
                }
                else {
                    // Constraint the edge
                    root->addConstraint(Constraint(b.getId(), initialAction.getLocation(), initialAction.timestamp, initialAction.timestamp + initialAction.duration));
                    // Constraint the opposite edge
                    root->addConstraint(Constraint(b.getId(), initialAction.endVertex->getEdge(initialAction.startVertex), initialAction.timestamp, initialAction.timestamp + initialAction.duration));
                    // Constraint the end vertex
                    root->addConstraint(Constraint(b.getId(), Location(initialAction.endVertex), initialAction.timestamp + initialAction.duration, initialAction.timestamp + initialAction.duration + TIME_AT_VERTEX));
                    // Constraint the start vertex
                    root->addConstraint(Constraint(b.getId(), Location(initialAction.startVertex), initialAction.timestamp, initialAction.timestamp + TIME_AT_VERTEX));
                }
            }
        }
    }

    #ifdef HIGHLEVEL_ANALYSIS_LOGS_ON
    logger.log("Finding initial paths\n");
    #endif
    
    try{
        root->setSolution(lowLevel.getAllPaths(graph, agents, root->getConstraints()));
    }
    catch (std::string exception){
        Error::log("Could not find initial solution. ERROR: " + exception + "\n");
        #ifndef EXPERIMENT
        return getGreedySolution(graph, agents, lowLevel, currentTime);
        #else
        exit(1);
        #endif
    }
    int minConflicts = std::numeric_limits<int>::max();
    auto minConflictsNode = root;
    /**
     * Insert Root to OPEN
     */
    std::priority_queue<std::shared_ptr<ConstraintTree>, std::vector<std::shared_ptr<ConstraintTree>>, ConstraintTree> open;
    open.push(root);
    /**
     * While OPEN not empty do
     */
    iterations = 0;
    while (open.size() > 0) {
        #ifndef EXPERIMENT
        auto timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin).count();
        if (timeDiff > 10000000){// If the solution takes more than 10 second
            auto& solution = minConflictsNode->getSolution();
            solution.finalize(agents);
            return solution;
        }
        #endif
        #ifdef HIGHLEVEL_ANALYSIS_LOGS_ON
        std::chrono::steady_clock::time_point iterationBegin = std::chrono::steady_clock::now();
        #endif

        if (++iterations == 5000000){
            Error::log("Max highlevel iterations reached!\n");
            exit(0);
        }
        #ifdef HIGHLEVEL_ANALYSIS_LOGS_ON
        (*logger.begin()) << "High level iteration: " << iterations << "\n"; logger.end();
        #endif
        /**
         * p <-- best node from OPEN (the node with the lowest solution cost)
         */
        std::shared_ptr<ConstraintTree> p = open.top();open.pop();
        #ifdef HIGHLEVEL_ANALYSIS_LOGS_ON
        (*logger.begin()) << "Constraints: " << p->getConstraints().size() << "\n"; logger.end();
        #endif
        #ifdef DEBUG_LOGS_ON
        Error::log("Popped this solution:\n");
        int aId = 0;
        for (auto pa : p->getSolution().paths){
            Error::log("agent" + std::to_string(aId++) + ": " + pa.toString() + "\n");
        }
        #endif
        /**
         * Validate the paths in P until a conflict occurs
         */
        blockGoalsForever(p->getSolution());
        std::vector<Conflict> conflicts = p->findConflicts();
        removeInfiniteBlocksOnGoals(p->getSolution());
        /**
         * If P has no conflicts then return P.solution
         */
        if (conflicts.size() == 0) {
            #ifdef HIGHLEVEL_ANALYSIS_LOGS_ON
            auto timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin).count();
            (*logger.begin()) << "Highlevel solution took " << timeDiff << "[µs]\n"; logger.end();
            #endif
            #ifdef DEBUG_LOGS_ON
            Error::log("Found solution with no conflicts\n");
            #endif
            // Prune floppy paths from solution
            Solution& solution = p->getSolution();
            solution.finalize(agents);
            return solution;
        }
        else if ((int)conflicts.size() < minConflicts){
            minConflictsNode = p;
            minConflicts = conflicts.size();
        }
        /**
         * Get one of the conflicts
         */
        Conflict c = getBestConflict(p, graph, agents, conflicts, lowLevel);
        // Conflict c = conflicts.front();
        #ifdef DEBUG_LOGS_ON
        Error::log(c.toString() + "\n");
        #endif
        /**
         * Foreach agent ai in C do
         */
        std::vector<int> agentIds = c.getAgentIds();
        int agentCount = agentIds.size();
        for (int i = 0; i < agentCount; ++i){
            int agentId = agentIds[i];
            /**
             * A <-- new node
             * A.constraints = p.constraints union (ai,v,t)
             */
            std::shared_ptr<ConstraintTree> a = std::make_shared<ConstraintTree>(agents.size());//TODO should we connect this to P or is it irrelevant in implementation?
            a->setConstraints(p->getConstraints());
            Constraint constraint = Constraint(
                agentId,
                c.getLocation(i),
                c.getTimeStart(),
                c.getTimeEnd()
            );
            
            a->addConstraint(constraint);
            /**
             * A.solution <-- P.solution
             * Update A.solution by invoking low level(ai)
             */
            Solution s = p->getSolution();
            try {
                Path newPath = lowLevel.getIndividualPath(graph, agents[agentId], a->getConstraints(agentId));
                #ifdef DEBUG_LOGS_ON
                Error::log(newPath.toString() + "\n");
                #endif
                s.paths[agentId] = newPath;
            }
            catch (std::string exception){
                #ifdef DEBUG_LOGS_ON
                Error::log(exception);
                #endif
                continue; // If the agent cant create a path to its goal with the new constraints, try adding the constraint to the other agent involved in the conflict
            }
            
            a->setSolution(s);
            /**
             * If A.cost < INF then insert A to OPEN
             */
            if (a->getCost() < std::numeric_limits<float>::infinity()) {
                open.push(a);
                #ifdef DEBUG_LOGS_ON
                Error::log("A cost is " + std::to_string(a->getCost()) + "\n");
                Error::log("A was pushed\n");
                #endif
            }
        }
        #ifdef HIGHLEVEL_ANALYSIS_LOGS_ON
        auto iterationTimeDiff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - iterationBegin).count();
        (*logger.begin()) << "High level iteration took " << iterationTimeDiff << "[µs]\n"; logger.end();
        #endif
    }
    // We did not find any solution (No possible solution)
    Error::log("ERROR: HighLevelCBS: No possible solution\n");
    #ifndef EXPERIMENT
    auto& solution = minConflictsNode->getSolution();
    solution.finalize(agents);
    return solution;
    #else
    exit(1);
    #endif
}

Conflict HighLevelCBS::getBestConflict(std::shared_ptr<ConstraintTree> node, std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents, std::vector<Conflict> conflicts, LowLevelCBS& lowLevel){
    if (conflicts.size() == 1) return conflicts.front();
    #ifdef DEBUG_LOGS_ON
    Error::log("Finding best conflict\n");
    #endif
    std::vector<Conflict> semiCardinalConflicts;
    for (auto c : conflicts){
        #ifdef DEBUG_LOGS_ON
        Error::log(c.toString() + "\n");
        #endif
        // Make a copy of the solution
        auto solution = node->getSolution();
        // It is a cardinal conflict if adding any of the constraints derived from the conflict
        // increase the cost.
        // It is a semi cardinal conflict if adding one of the derived constraints increase the cost
        // We see: If we find the condition for semi cardinal to be true > 1 it is a cardinal.
        bool semiCardinal = false;
        std::vector<int> agentIds = c.getAgentIds();
        int agentCount = agentIds.size();
        for (int i = 0; i < agentCount; ++i){
            int agentId = agentIds[i];
            // Get the derived constraint for the current agent from the conflict
            Constraint constraint = Constraint(
                agentId,
                c.getLocation(i),
                c.getTimeStart() - 1,
                c.getTimeEnd() + 1
            );
            // Get a container of the current constraints union the new constraint
            auto constraints = node->getConstraints(agentId);
            constraints.push_back(constraint);
            // Get the new path from low level so we can see if the cost increases
            float currentCost = solution.paths[agentId].cost;
            try{
                Path newPath = lowLevel.getIndividualPath(graph, agents[agentId], constraints);
                bool increasesCost = newPath.cost > currentCost;

                if (increasesCost){
                    if (semiCardinal){
                        #ifdef DEBUG_LOGS_ON
                        Error::log("Found cardinal conflict\n");
                        #endif
                        return c;
                    }
                    else {
                        semiCardinalConflicts.push_back(c);
                        semiCardinal = true;
                    }
                }
            }catch(std::string exception){
                #ifdef DEBUG_LOGS_ON
                Error::log(exception + "\n");
                #endif
                continue;
            }
        }
    }
    #ifdef DEBUG_LOGS_ON
    Error::log("Returning semi cardinal / front\n");
    #endif
    // Return a semi cardinal if one was found, otherwise return a random conflict
    return semiCardinalConflicts.empty() ? conflicts.front() : semiCardinalConflicts.front();
}

void HighLevelCBS::blockGoalsForever(Solution& solution){
    float maxPathCost = 0;
    for (auto& p : solution.paths){
        maxPathCost = std::max(p.cost, maxPathCost);
    }
    for (auto& p : solution.paths){
        p.actions.push_back(
            Action(p.cost, p.actions.back().endVertex, p.actions.back().endVertex,
                maxPathCost - (p.actions.back().timestamp + p.actions.back().duration))
        );
    }
}

void HighLevelCBS::removeInfiniteBlocksOnGoals(Solution& solution){
    for (auto& p : solution.paths){
        p.actions.erase(p.actions.end());
    }
}

Solution HighLevelCBS::getGreedySolution(std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents, LowLevelCBS& lowLevel, float currentTime){
    Solution solution;
    #ifdef DEBUG_LOGS_ON
    Error::log("Getting greedy solution\n");
    #endif
    solution.paths = lowLevel.getAllPaths(graph, agents, std::vector<std::vector<Constraint>>(agents.size()));// Low level with no constraints is greedy A* solution
    solution.finalize(agents);
    
    // Limit the paths to 1 action
    for (auto& p : solution.paths){
        if (p.actions.size() > 1){
            p.actions = {p.actions.front()};
        }
    }
    
    #ifdef DEBUG_LOGS_ON
    Error::log("Current time = " + std::to_string(currentTime) + "\n");
    for(auto& p : solution.paths){
        Error::log(p.toString() + "\n");
    }
    #endif
    return solution;
}