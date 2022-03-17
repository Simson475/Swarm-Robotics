#include "HighLevelCBS.hpp"

Solution* HighLevelCBS::findSolution(Graph graph, std::vector<AgentInfo> agentInfo, LowLevelCBS lowLevel){
    ConstraintTree root{};
    root.children = {};//Root.constraints = {}
    root.solution = lowLevel.getAllPaths(agentInfo);//Root.solution = find individual paths by the low level
    //Do nothing, .getSICCost() does this..//Root.cost = SIC(Root.solution)
    std::priority_queue<ConstraintTree> open; open.push(root);//insert Root to OPEN
    
    while (open.size() > 0) {//while OPEN not empty do
        ConstraintTree p = open.top();open.pop();//p <-- best node from OPEN // lowest solution cost
        std::vector<Conflict> conflicts = p.getConflicts();//Validate the paths in P until a conflict occurs
        if (conflicts.size() == 0) {//if P has no conflicts then
            return p.solution;//return P.solution
        }
        Conflict &c = conflicts.front();//C <-- first conflict (ai, aj, v, t) in P // Replace with ICBS conflict priorization later
        for(AgentInfo &agent : c.getAgents()){//foreach agent ai in C do
            ConstraintTree a {};//A <-- new node
            a.constraints = p.constraints; a.constraints.emplace_back(new Constraint(agent, c.timestampStart, c.timestampEnd));//A.constraints <-- P.constraints + (ai,v,t)
            a.solution = p.solution;//A.solution <-- P.solution
            //TODO check this line
            a.solution.setPath(agent, lowLevel.getIndividualPath(agent));//Update A.solution by invoking low level(ai)
            //Do nothing, A.getCost does this.. //A.cost = SIC(A.solution)
            if (a.getSICCost() < INFINITY) {//if A.cost < INF then//A solution was found
                open.push(a);//Insert A to OPEN
            }
        }
    }
    return root.solution;
}


// for(Agent *agent : root->solution->agents){
//     agent->getBot()->receivedWaypointPlan = agent->getPath().asWaypointPlan();
//     agent->getBot()->setWaypointPlan(agent->getPath().asWaypointPlan());
// }