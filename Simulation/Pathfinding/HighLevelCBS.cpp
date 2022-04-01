#include "HighLevelCBS.hpp"

Solution HighLevelCBS::findSolution(std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents, LowLevelCBS lowLevel){
    for(auto a : agents){
        Error::log("Goal: ");
        Error::log(std::to_string(a.getGoal()->getId()));
        Error::log("\n");
    }
    /**
     * Root.constraints = {}
     * Root.solution = find individual paths by the low level
     * Root.cost = SIC(Root.solution)
     */
    std::shared_ptr<ConstraintTree> root = std::make_shared<ConstraintTree>();
    root->setSolution(lowLevel.getAllPaths(graph, agents, std::vector<Constraint>{}), agents);
    /**
     * Insert Root to OPEN
     */
    std::priority_queue<std::shared_ptr<ConstraintTree>> open;
    open.push(root);
    /**
     * While OPEN not empty do
     */
    int iterations = 0;
    while (open.size() > 0) {
        iterations++;
        if (iterations == 3){
            exit(1);
        }
        /**
         * p <-- best node from OPEN (the node with the lowest solution cost)
         */
        std::shared_ptr<ConstraintTree> p = open.top();open.pop();
        /**
         * Validate the paths in P until a conflict occurs
         */
        // Error::log("finding Conflicts now:\n");
        std::vector<Conflict> conflicts = p->findConflicts();
        /**
         * If P has no conflicts then return P.solution
         */
        for(auto p : p->getSolution().paths){
            std::cout << "p:" << p.toString() << "\n";
        }
        std::cout << conflicts.size() << " conflicts found\n";
        if (conflicts.size() == 0) {
            Error::log("After ");
            Error::log(std::to_string(iterations));
            Error::log("iterations, we found a solution!\n");
            return p->getSolution();
        }
        /**
         * Get one of the conflicts
         */
        Conflict &c = conflicts.front();
        /**
         * Foreach agent ai in C do
         */
        for(int agentId : c.getAgentIds()){
            AgentInfo agent = agents[agentId];
            /**
             * A <-- new node
             * A.constraints = p.constraints union (ai,v,t)
             */
            std::shared_ptr<ConstraintTree> a = std::make_shared<ConstraintTree>();//TODO should we connect this to P or is it irrelevant in implementation?
            a->constraints = p->constraints;

            Constraint constraint = Constraint(
                agent,
                c.getLocation(),
                c.getTimeStart(),
                c.getTimeEnd()
            );
            std::cout << constraint.toString() << "\n";
            a->constraints.emplace_back(constraint);
            /**
             * A.solution <-- P.solution
             */
            a->setSolution(p->getSolution());
            /**
             * Update A.solution by invoking low level(ai)
             */
            Solution s = a->getSolution();
            Path newPath = lowLevel.getIndividualPath(graph, agent, a->constraints);
            s.paths[agent.getId()] = newPath;
            a->setSolution(s);
            /**
             * If A.cost < INF then insert A to OPEN
             */
            if (a->getCost() < INFINITY) {
                open.push(a);
            }
        }
    }
    // We did not find any solution (No possible solution)
    Error::log("ERROR: HighLevelCBS: No possible solution\n");
    exit(1);
}