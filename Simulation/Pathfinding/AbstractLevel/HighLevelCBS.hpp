class HighLevelCBS;
#ifndef HIGHLEVEL_CBS_HPP
#define HIGHLEVEL_CBS_HPP

//#include "argos_wrapper/argos_wrapper.hpp"
//#include "map_structure.hpp"
#include "ConstraintTree.hpp"
//#include "SingleThreadBotCBS.hpp"
#include <vector>
#include <queue>
#include "Conflict.hpp"
#include "Solution.hpp"
//#include "TestController.hpp"
#include "Graph.hpp"
#include "AgentInfo.hpp"
#include "LowLevelCBS.hpp"

class HighLevelCBS {    
public:
    // Singleton
    static HighLevelCBS &get_instance() {
        static HighLevelCBS instance;
        return instance;
    }
    Solution* findSolution(Graph, std::vector<AgentInfo>, LowLevelCBS);
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
};

#endif