#include "HighLevelCBS.hpp"

//Root.constraints = {}
//Root.solution = find individual paths by the low level
//Root.cost = SIC(Root.solution)
//insert Root to OPEN
//while OPEN not empty do
    //p <-- best node from OPEN // lowest solution cost
    //Validate the paths in P until a conflict occurs
    //if P has no conflicts then
        //return P.solution
    //C <-- first conflict (ai, aj, v, t) in P /* Replace with ICBS conflict priorization later */
    /* INSERT MA-CBS here later */
    //foreach agent ai in C do
        //A <-- new node
        //A.constraints <-- P.constraints + (ai,v,t)
        //A.solution <-- P.solution
        //Update A.solution by invoking low level(ai)
        //A.cost = SIC(A.solution)
        //if A.cost < INF then//A solution was found
            //Insert A to OPEN

// int HighLevelCBS::SumOfIndividualCosts(Solution solution){
//     return solution.cost;
// }

// std::vector<Conflict> HighLevelCBS::findConflicts(ConstraintTree& ctNode){
//     std::vector<Conflict> temp{};
//     //TODO implement
//     return temp;
// }

std::vector<TestController*> HighLevelCBS::getControllers(){
    if (botAmount != 0){ return controllers; }
    // Get controllers
    auto &tBotMap = argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    controllers = {};
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        TestController* controller = dynamic_cast<TestController*>(&(pcBot->GetControllableEntity().GetController()));
        controllers.push_back(controller);
    }
    
    return controllers;
}

Solution* HighLevelCBS::findAllPathsByLowLevel(){
//     //TODO we can only do this if all against have NO conflicts/constraints. (Relevant for -ma-CBS only)
    Solution* solution = new Solution();
    Map_Structure map = Map_Structure::get_instance();
    std::vector<Agent*> allAgents;
    auto botList = getControllers();
    for(TestController* bot : botList){
        Agent* agent = new Agent();
        agent->setBot(bot);
        auto plan = bot->findOptimalPath();
        agent->createPath(plan);
        allAgents.push_back(agent);
    };
    solution->agents = allAgents;
    return solution;
}

Solution* HighLevelCBS::findSolution(){
    ConstraintTree* root = new ConstraintTree();
    root->children = {};//Root.constraints = {}
    root->solution = findAllPathsByLowLevel();//Root.solution = find individual paths by the low level

    for(Agent *agent : root->solution->agents){
        agent->getBot()->receivedWaypointPlan = agent->getPath().asWaypointPlan();
    }
    
    return root->solution;
//     root.cost = SumOfIndividualCosts(root.solution);//Root.cost = SIC(Root.solution)
//     //std::priority_queue<ConstraintTree> open; open.push(root);//insert Root to OPEN
    
//     // while (open.size() > 0) {//while OPEN not empty do
//     //     ConstraintTree p = open.top();open.pop();//p <-- best node from OPEN // lowest solution cost
//     //     std::vector<Conflict> conflicts = findConflicts(p);//Validate the paths in P until a conflict occurs
//     //     if (conflicts.size() == 0) {//if P has no conflicts then
//     //         return p.solution;//return P.solution
//     //     }
//     //     Conflict &c = conflicts.front();//C <-- first conflict (ai, aj, v, t) in P // Replace with ICBS conflict priorization later
//     //     // INSERT MA-CBS here later
//     //     for(Agent &agent : c.agents){//foreach agent ai in C do
//     //         ConstraintTree a {};//A <-- new node
//     //         a.constraints = p.constraints; a.constraints.emplace_back(new Constraint(agent, c.timestampStart, c.timestampEnd));//A.constraints <-- P.constraints + (ai,v,t)
//     //         a.solution = p.solution;//A.solution <-- P.solution
//     //         agent.recreatePath();//Update A.solution by invoking low level(ai)
//     //         a.cost = SumOfIndividualCosts(a.solution);//A.cost = SIC(A.solution)
//     //         if (a.cost < INFINITY) {//if A.cost < INF then//A solution was found
//     //             open.push(a);//Insert A to OPEN
//     //         }
//     //     }
//     // }
//     return root.solution;
}
