#include "HighLevelCBS.hpp"

Solution HighLevelCBS::findSolution(std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents, LowLevelCBS lowLevel){
    auto logger = Logger::get_instance();
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    /**
     * Root.constraints = {}
     * Root.solution = find individual paths by the low level
     * Root.cost = SIC(Root.solution)
     */
    std::shared_ptr<ConstraintTree> root = std::make_shared<ConstraintTree>();
    // std::cout << "initial paths\n";
    if (Logger::enabled) {
        logger.log("Finding initial paths\n");
    }
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
        std::chrono::steady_clock::time_point iterationBegin = std::chrono::steady_clock::now();

        if (++iterations == 10000){
            Error::log("Max highlevel iterations reached!\n");
            exit(0);
        }
        if (Logger::enabled) {
            (*logger.begin()) << "High level iteration: " << iterations << "\n"; logger.end();
        }
        /**
         * p <-- best node from OPEN (the node with the lowest solution cost)
         */
        std::shared_ptr<ConstraintTree> p = open.top();open.pop();
        if (Logger::enabled) {
            (*logger.begin()) << "Constraints: " << p->getConstraints().size() << "\n"; logger.end();
        }
        // Error::log("Popped this solution:\n");
        // for (auto pa : p->getSolution().paths){
        //     Error::log(pa.toString() + "\n");
        // }
        std::cout << "Popped this solution:\n";
        for (auto pa : p->getSolution().paths){
            std::cout << pa.toString() << "\n";
        }
        /**
         * Validate the paths in P until a conflict occurs
         */
        std::vector<Conflict> conflicts = p->findConflicts();
        // std::cout << "Found these conflicts:\n";
        // std::cout << conflicts.size() << "\n";
        // for (auto conf : conflicts){
        //     std::cout << conf.toString() << "\n";
        // }
        /**
         * If P has no conflicts then return P.solution
         */
        if (conflicts.size() == 0) {
            if (Logger::enabled) {
                auto timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin).count();
                (*logger.begin()) << "Highlevel solution took " << timeDiff << "[µs]\n"; logger.end();
            }
            return p->getSolution();
        }
        /**
         * Get one of the conflicts
         */
        // Conflict &c = conflicts.front();
        // std::cout << "Finding best conflict..\n";
        Conflict c = getBestConflict(p, graph, agents, conflicts, lowLevel);
        Error::log(c.toString() + "\n");
        std::cout << c.toString() + "\n";
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
            //std::cout << c.getLocation().toString() << "<-- get location  get time start-->"<< c.getTimeStart()<< "\n"
            //<< "Constraints.size: " << a->constraints.size() << "\n";
            Constraint constraint = Constraint(
                agentId,
                c.getLocation(),
                c.getTimeStart(),
                c.getTimeEnd()
            );
            // Error::log(constraint.toString() + "\n");
            std::cout << constraint.toString() << "\n";
            // if (agentId == c.getAgentIds()[0]){
            //     std::cout << "X\n";
            //     for (auto pa : p->getSolution().paths){
            //         std::cout << pa.toString() << "\n";
            //     }
            // std::cout << constraint.toString() << "\n" << agentId << "\n";
            // }
            a->addConstraint(constraint);
            // for (auto constr : a->constraints){
            //     std::cout << constr.toString() << "\n";
            // }
            /**
             * A.solution <-- P.solution
             * Update A.solution by invoking low level(ai)
             */
            Solution s = p->getSolution();
            // std::cout << "individual path\n";
            
            Path newPath = lowLevel.getIndividualPath(graph, agents[agentId], a->getConstraints(agentId));
            s.paths[agentId] = newPath;
            a->setSolution(s);
            // for (auto pa : a->getSolution().paths){
            //     std::cout << pa.toString() << "\n";
            // }
            /**
             * If A.cost < INF then insert A to OPEN
             */
            if (a->getCost() < std::numeric_limits<float>::infinity()) {
                open.push(a);
            }
        }

        if (Logger::enabled) {
            auto iterationTimeDiff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - iterationBegin).count();
            (*logger.begin()) << "High level iteration took " << iterationTimeDiff << "[µs]\n"; logger.end();
        }
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