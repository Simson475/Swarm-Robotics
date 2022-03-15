class HighLevelCBS;
#ifndef HIGHLEVEL_CBS_HPP
#define HIGHLEVEL_CBS_HPP

#include "argos_wrapper/argos_wrapper.hpp"
#include "map_structure.hpp"
#include "ConstraintTree.hpp"
#include "SingleThreadBotCBS.hpp"
#include <queue>
#include "Conflict.hpp"
#include "Solution.hpp"
#include "TestController.hpp"

class HighLevelCBS {
private:
    int botAmount = 0;
    std::vector<TestController*> controllers;

    //int SumOfIndividualCosts(Solution solution);
    //std::vector<Conflict> findConflicts(ConstraintTree& ctNode);
    Solution* findAllPathsByLowLevel();
    std::vector<TestController*> getControllers();
    
public:
    HighLevelCBS() = default;
    //ensures that the class is created only once
    static HighLevelCBS &get_instance() {
        static HighLevelCBS instance;
        return instance;
    }
    Solution* findSolution();
};

#endif