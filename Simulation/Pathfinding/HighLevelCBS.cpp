#include "HighLevelCBS.hpp"

Solution HighLevelCBS::findSolution(std::shared_ptr<Graph> graph, std::vector<AgentInfo> agents, LowLevelCBS lowLevel){
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
    std::priority_queue<std::shared_ptr<ConstraintTree>, std::vector<std::shared_ptr<ConstraintTree>>, ConstraintTree> open;
    open.push(root);
    std::vector<std::shared_ptr<ConstraintTree>> cts;
    cts.push_back(root);
    /**
     * While OPEN not empty do
     */
    int iterations = 0;
    while (open.size() > 0) {
        if (++iterations == 1000){
            Error::log("Max highlevel iterations reached!\n");
            exit(0);
        }
        /**
         * p <-- best node from OPEN (the node with the lowest solution cost)
         */
        std::shared_ptr<ConstraintTree> p = open.top();open.pop();
        // std::cout << "Popped this solution:\n";
        // for (auto pa : p->getSolution().paths){
        //     std::cout << pa.toString() << "\n";
        // }
        /**
         * Validate the paths in P until a conflict occurs
         */
        std::vector<Conflict> conflicts = p->findConflicts();
        // std::cout << "Found these conflicts:\n";
        // std::cout << conflicts.size() << "\n";
        // for (auto conf : conflicts){
        //     std::cout << conf.toString();
        // }
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
        for(int agentId : c.getAgentIds()){
            AgentInfo agent = agents[agentId];
            /**
             * A <-- new node
             * A.constraints = p.constraints union (ai,v,t)
             */
            std::shared_ptr<ConstraintTree> a = std::make_shared<ConstraintTree>();//TODO should we connect this to P or is it irrelevant in implementation?
            a->constraints = p->constraints;
            //std::cout << c.getLocation().toString() << "<-- get location  get time start-->"<< c.getTimeStart()<< "\n"
            //<< "Constraints.size: " << a->constraints.size() << "\n";
            Constraint constraint = Constraint(
                agent,
                c.getLocation(),
                c.getTimeStart() - 1,
                c.getTimeEnd() + 1
            );
            // if (agentId == c.getAgentIds()[0]){
            //     std::cout << "X\n";
            //     for (auto pa : p->getSolution().paths){
            //         std::cout << pa.toString() << "\n";
            //     }
            //     std::cout << constraint.toString() << "\n";
            // }
            a->constraints.push_back(constraint);
            // for (auto constr : a->constraints){
            //     std::cout << constr.toString() << "\n";
            // }
            /**
             * A.solution <-- P.solution
             * Update A.solution by invoking low level(ai)
             */
            Solution s = p->getSolution();
            Path newPath = lowLevel.getIndividualPath(graph, agent, a->constraints);
            s.paths[agent.getId()] = newPath;
            a->setSolution(s);
            // for (auto pa : a->getSolution().paths){
            //     std::cout << pa.toString() << "\n";
            // }
            /**
             * If A.cost < INF then insert A to OPEN
             */
            if (a->getCost() < INFINITY) {
                for (auto ct : cts){
                    if (a->constraints.size() == ct->constraints.size()){
                        bool equals = true;
                        int siz = a->constraints.size();
                        for (int i = 0; i < siz; ++i){
                            if (!(a->constraints[i] == ct->constraints[i])){
                                equals = false;
                                break;
                            }
                        }
                        if (equals){
                            std::cout << "DUBLICATES!\n";
                            exit(1);
                        }
                    }
                }
                open.push(a);
                cts.push_back(a);
            }
        }
    }
    // We did not find any solution (No possible solution)
    Error::log("ERROR: HighLevelCBS: No possible solution\n");
    exit(1);
}