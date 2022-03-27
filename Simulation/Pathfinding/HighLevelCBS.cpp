#include "HighLevelCBS.hpp"

Solution HighLevelCBS::findSolution(std::shared_ptr<Graph> graph, std::vector<std::shared_ptr<Agent>> agents, LowLevelCBS lowLevel){
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
    while (open.size() > 0) {
        /**
         * p <-- best node from OPEN (the node with the lowest solution cost)
         */
        std::shared_ptr<ConstraintTree> p = open.top();open.pop();
        /**
         * Validate the paths in P until a conflict occurs
         */
        std::vector<Conflict> conflicts = p->findConflicts();
        /**
         * If P has no conflicts then return P.solution
         */
        if (conflicts.size() == 0) {
            return p->getSolution();
        }
        /**
         * Get one of the conflicts
         */
        Conflict &c = conflicts.front();
        /**
         * Foreach agent ai in C do
         */
        for(std::shared_ptr<Agent> agent : c.getAgents()){
            /**
             * A <-- new node
             * A.constraints = p.constraints union (ai,v,t)
             */
            std::shared_ptr<ConstraintTree> a = std::make_shared<ConstraintTree>();//TODO should we connect this to P or is it irrelevant in implementation?
            a->constraints = p->constraints;
            a->constraints.emplace_back(Constraint(
                agent,
                c.getLocation(),
                c.getTimeStart(),
                c.getTimeEnd()
            ));
            /**
             * A.solution <-- P.solution
             */
            a->setSolution(p->getSolution());
            /**
             * Update A.solution by invoking low level(ai)
             */
            Solution s = a->getSolution();
            
            Error::log(">");
            Path newPath = lowLevel.getIndividualPath(graph, agent, a->constraints);
            Error::log(".3\n");
            Error::log(std::to_string(agent->getId()));
            Error::log("|");
            Error::log(std::to_string(s.paths.size()));
            s.paths[agent->getId()] = newPath;
            Error::log("\n9");
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