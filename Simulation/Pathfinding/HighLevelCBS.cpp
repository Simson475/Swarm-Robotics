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

int HighLevelCBS::SumOfIndividualCosts(Solution solution){
    return solution.cost;
}
std::vector<Conflict> HighLevelCBS::findConflicts(ConstraintTree ctNode){
    std::vector<Conflict> temp{};
    return temp;
}

Solution HighLevelCBS::findAllPathsByLowLevel(){
    Solution solution{};
    Map_Structure map = Map_Structure::get_instance();
    std::vector<Agent> allAgents;
    for(auto  &bot : botList){
        Agent agent{};
        auto plan = bot.findOptimalPath();
        agent.createPath(plan);
        allAgents.push_back(agent);
    };
    solution.agents = allAgents;
    return solution;
}

Solution HighLevelCBS::findSolution(){
    root.constraints = {};//Root.constraints = {}
    root.solution = findAllPathsByLowLevel();//Root.solution = find individual paths by the low level
    root.cost = SumOfIndividualCosts(root.solution);//Root.cost = SIC(Root.solution)
    std::priority_queue<ConstraintTree> open; open.push(root);//insert Root to OPEN
    
    while (open.size() > 0) {//while OPEN not empty do
        ConstraintTree p = open.top();open.pop();//p <-- best node from OPEN // lowest solution cost
        std::vector<Conflict> conflicts = findConflicts(p);//Validate the paths in P until a conflict occurs
        if (conflicts.size()) {//if P has no conflicts then
            return p.solution;//return P.solution
        }
        Conflict &c = conflicts.front();//C <-- first conflict (ai, aj, v, t) in P /* Replace with ICBS conflict priorization later */
        /* INSERT MA-CBS here later */
        for(Agent &agent : c.agents){//foreach agent ai in C do
            ConstraintTree A {};//A <-- new node
            //TODO copy and add        A.constraints = P.constraints;//A.constraints <-- P.constraints + (ai,v,t)
            //A.solution <-- P.solution
            //Update A.solution by invoking low level(ai)
            //A.cost = SIC(A.solution)
            //if A.cost < INF then//A solution was found
                //Insert A to OPEN
        }
    }
}