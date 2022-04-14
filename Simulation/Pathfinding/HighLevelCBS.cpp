#include "HighLevelCBS.hpp"

Solution HighLevelCBS::findSolution(std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents, LowLevelCBS lowLevel){
    #ifdef EXPERIMENT
    Logger& logger = Logger::get_instance();
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    Error::log("Test\n");
    #endif
    /**
     * Root.constraints = {}
     * Root.solution = find individual paths by the low level
     * Root.cost = SIC(Root.solution)
     */
    std::shared_ptr<ConstraintTree> root = std::make_shared<ConstraintTree>();
    // Set initial constraints to avoid conflicts on initial actions
    for (auto a : agents){
        for (auto b : agents){
            if (a.getId() != b.getId()){
                auto initialAction = a.getCurrentAction();
                if (initialAction.isWaitAction()){
                    // Constraint the initial vertex
                    root->addConstraint(Constraint(b.getId(), initialAction.getLocation(), initialAction.timestamp, initialAction.timestamp + initialAction.duration + TIME_AT_VERTEX));
                }
                else {
                    // Constraint the edge action and the vertex it arrives at
                    root->addConstraint(Constraint(b.getId(), initialAction.getLocation(), initialAction.timestamp, initialAction.timestamp + initialAction.duration));
                    root->addConstraint(Constraint(b.getId(), Location(initialAction.endVertex), initialAction.timestamp, initialAction.timestamp + initialAction.duration + TIME_AT_VERTEX));
                }
            }
        }
    }

    #ifdef EXPERIMENT
    if (Logger::enabled) {
        logger.log("Finding initial paths\n");
    }
    #endif
    root->setSolution(lowLevel.getAllPaths(graph, agents, std::vector<Constraint>{}), agents);
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
        #ifdef EXPERIMENT
        std::chrono::steady_clock::time_point iterationBegin = std::chrono::steady_clock::now();
        #endif

        if (++iterations == 5000000){
            Error::log("Max highlevel iterations reached!\n");
            exit(0);
        }
        #ifdef EXPERIMENT
        if (Logger::enabled) {
            (*logger.begin()) << "High level iteration: " << iterations << "\n"; logger.end();
        }
        #endif
        /**
         * p <-- best node from OPEN (the node with the lowest solution cost)
         */
        std::shared_ptr<ConstraintTree> p = open.top();open.pop();
        #ifdef EXPERIMENT
        if (Logger::enabled) {
            (*logger.begin()) << "Constraints: " << p->getConstraints().size() << "\n"; logger.end();
        }
        #endif
        #ifdef DEBUG_LOGS_ON
        Error::log("Popped this solution:\n");
        for (auto pa : p->getSolution().paths){
            Error::log(pa.toString() + "\n");
        }
        #endif
        /**
         * Validate the paths in P until a conflict occurs
         */
        std::vector<Conflict> conflicts = p->findConflicts();
        /**
         * If P has no conflicts then return P.solution
         */
        if (conflicts.size() == 0) {
            #ifdef EXPERIMENT
            if (Logger::enabled) {
                auto timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin).count();
                (*logger.begin()) << "Highlevel solution took " << timeDiff << "[µs]\n"; logger.end();
            }
            #endif
            return p->getSolution();
        }
        /**
         * Get one of the conflicts
         */
        Conflict c = getBestConflict(p, graph, agents, conflicts, lowLevel);
        #ifdef DEBUG_LOGS_ON
        Error::log(c.toString() + "\n");
        #endif
        // std::cout << c.toString() + "\n";
        /**
         * Foreach agent ai in C do
         */
        for(int agentId : c.getAgentIds()){
            /**
             * A <-- new node
             * A.constraints = p.constraints union (ai,v,t)
             */
            std::shared_ptr<ConstraintTree> a = std::make_shared<ConstraintTree>();//TODO should we connect this to P or is it irrelevant in implementation?
            a->setConstraints(p->getConstraints());
            Constraint constraint = Constraint(
                agentId,
                c.getLocation(),
                c.getTimeStart(),
                c.getTimeEnd()
            );
            
            // Only add the constraint if the agent can avoid it
            auto currentAgentAction = agents[agentId].getCurrentAction();
            bool constraintIsOnEndVertex = constraint.location == currentAgentAction.endVertex;
            bool constraintIsOnInitialWaitAction = currentAgentAction.isWaitAction() && constraintIsOnEndVertex;
            bool constraintIsBeforeDeltaAfterAction = constraint.timeStart <= (currentAgentAction.timestamp + currentAgentAction.duration + TIME_AT_VERTEX);
            bool constraintIsOnInitialEdgeAction = ! currentAgentAction.isWaitAction()
             && constraint.location == currentAgentAction.getLocation();
            bool constraintIsBeforeActionEnds = constraint.timeStart <= (currentAgentAction.timestamp + currentAgentAction.duration);

            if ((constraintIsOnInitialWaitAction && constraintIsBeforeDeltaAfterAction)
             || ((constraintIsOnInitialEdgeAction && constraintIsBeforeActionEnds) || (constraintIsOnEndVertex && constraintIsBeforeDeltaAfterAction))
            ){
                break; // This agent has no way of avoiding the constraint, so it should not be added.
            }
            a->addConstraint(constraint);
            #ifdef DEBUG_LOGS_ON
            Error::log("A constraints for agent: \n");
            for (auto constr : a->getConstraints(agentId)){
                Error::log(constr.toString() + "\n");
            }
            #endif
            /**
             * A.solution <-- P.solution
             * Update A.solution by invoking low level(ai)
             */
            Solution s = p->getSolution();
            // std::cout << "individual path\n";
            
            Path newPath = lowLevel.getIndividualPath(graph, agents[agentId], a->getConstraints(agentId));
            #ifdef DEBUG_LOGS_ON
            Error::log(newPath.toString() + "\n");
            #endif
            s.paths[agentId] = newPath;
            a->setSolution(s);
            /**
             * If A.cost < INF then insert A to OPEN
             */
            if (a->getCost() < std::numeric_limits<float>::infinity()) {
                open.push(a);
                #ifdef DEBUG_LOGS_ON
                Error::log("A was pushed\n");
                #endif
            }
        }
        #ifdef EXPERIMENT
        if (Logger::enabled) {
            auto iterationTimeDiff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - iterationBegin).count();
            (*logger.begin()) << "High level iteration took " << iterationTimeDiff << "[µs]\n"; logger.end();
        }
        #endif
    }
    // We did not find any solution (No possible solution)
    Error::log("ERROR: HighLevelCBS: No possible solution\n");
    exit(1);
}

Conflict HighLevelCBS::getBestConflict(std::shared_ptr<ConstraintTree> node, std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents, std::vector<Conflict> conflicts, LowLevelCBS lowLevel){
    if (conflicts.size() == 1) return conflicts.front();

    std::vector<Conflict> semiCardinalConflicts;
    for (auto c : conflicts){
        // Make a copy of the solution
        auto solution = node->getSolution();
        // It is a cardinal conflict if adding any of the constraints derived from the conflict
        // increase the cost.
        // It is a semi cardinal conflict if adding one of the derived constraints increase the cost
        // We see: If we find the condition for semi cardinal to be true > 1 it is a cardinal.
        bool semiCardinal = false;
        for (auto agentId : c.getAgentIds()){
            // Get the derived constraint for the current agent from the conflict
            Constraint constraint = Constraint(
                agentId,
                c.getLocation(),
                c.getTimeStart() - 1,
                c.getTimeEnd() + 1
            );
            // Get a container of the current constraints union the new constraint
            auto constraints = node->getConstraints(agentId);
            constraints.push_back(constraint);
            // Get the new path from low level so we can see if the cost increases
            int currentCost = solution.paths[agentId].cost;
            Path newPath = lowLevel.getIndividualPath(graph, agents[agentId], constraints);
            bool increasesCost = newPath.cost > currentCost;

            if (increasesCost){
                if (semiCardinal){
                    return c;
                }
                else {
                    semiCardinalConflicts.push_back(c);
                    semiCardinal = true;
                }
            }
        }
    }
    // Return a semi cardinal if one was found, otherwise return a random conflict
    return semiCardinalConflicts.empty() ? conflicts.front() : semiCardinalConflicts.front();
}