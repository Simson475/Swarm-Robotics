#include "HighLevelCBS.hpp"

Solution HighLevelCBS::findSolution(Graph* graph, std::vector<AgentInfo> agentInfo, LowLevelCBS lowLevel){
    /**
     * Root.constraints = {}
     * Root.solution = find individual paths by the low level
     * Root.cost = SIC(Root.solution)
     */
    
    std::shared_ptr<ConstraintTree> root = std::make_shared<ConstraintTree>();
    Error::log("\n1");
    root->setSolution(lowLevel.getAllPaths(graph, agentInfo), agentInfo);//Root.solution = find individual paths by the low level
    Error::log("2");
    /**
     * Insert Root to OPEN
     */
    std::priority_queue<std::shared_ptr<ConstraintTree>> open;
    open.push(root);
    
    Error::log("3");
    /**
     * While OPEN not empty do
     */
    while (open.size() > 0) {
        /**
         * p <-- best node from OPEN (the node with the lowest solution cost)
         */
    
    Error::log("4");
        std::shared_ptr<ConstraintTree> p = open.top();open.pop();
        /**
         * Validate the paths in P until a conflict occurs
         */
    
    Error::log("5");
        std::vector<Conflict> conflicts = p->findConflicts();
        Error::log("6");
        /**
         * If P has no conflicts then return P.solution
         */
        if (conflicts.size() == 0) {
            return p->getSolution();
        }
        Error::log("6.2");
        /**
         * Get one of the conflicts
         */
        Conflict &c = conflicts.front();
        /**
         * Foreach agent ai in C do
         */
         Error::log("7");
        for(AgentInfo &agent : c.getAgents()){
            /**
             * A <-- new node
             * A.constraints = p.constraints union (ai,v,t)
             */
            Error::log("7.1");
            std::shared_ptr<ConstraintTree> a = std::make_shared<ConstraintTree>();//TODO should we connect this to P or is it irrelevant in implementation?
            std::shared_ptr<Constraint> constraint = std::make_shared<Constraint>();
            constraint->agent = std::make_unique<AgentInfo>(agent);
            constraint->timeStart = c.getTimeStart();
            constraint->timeEnd = c.getTimeEnd();
            a->constraints = p->constraints;
            Error::log("7.2");
            a->constraints.emplace_back(constraint);
            /**
             * A.solution <-- P.solution
             */
            Error::log(".3");
            a->setSolution(p->getSolution());
            Error::log("8");
            /**
             * Update A.solution by invoking low level(ai)
             */
            Solution s = a->getSolution();
            
            Error::log("8.2");
            Path newPath = lowLevel.getIndividualPath(graph, agent);
            Error::log(".3\n");
            Error::log(std::to_string(agent.getId()));
            Error::log("|");
            Error::log(std::to_string(s.paths.size()));
            s.setPath(agent, newPath);
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
}

bool HighLevelCBS::requestSolution(){
    if (!true){//TODO insert correct logic
        return false;
    }
    auto g = getGraph();
    auto a = getAgentInfo();
    auto ll = LowLevelCBS::get_instance();
    findSolution(g, a, ll);//TODO do we want to catch solution?
    return true;
}

Graph* HighLevelCBS::getGraph(){
    return &Map_Structure::get_instance();
}
std::vector<AgentInfo> HighLevelCBS::getAgentInfo(){
    auto agents = getAgents();
    size_t agentCount = agents.size();
    std::vector<AgentInfo> agentInfo{agentCount};
    for(size_t i = 0; i < agentCount; ++i){
        agentInfo[i].setId(i);
    }
    return agentInfo;
}
std::vector<Agent> HighLevelCBS::getAgents(){
    auto &tBotMap = argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    std::vector<Agent> agents{tBotMap.size()};
    int i = 0;
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        TestController* controller = dynamic_cast<TestController*>(&(pcBot->GetControllableEntity().GetController()));
        agents[i++].setBot(controller);
    }
    
    return agents;
}

// for(Agent *agent : root->solution->agents){
//     agent->getBot()->receivedWaypointPlan = agent->getPath().asWaypointPlan();
//     agent->getBot()->setWaypointPlan(agent->getPath().asWaypointPlan());
// }
