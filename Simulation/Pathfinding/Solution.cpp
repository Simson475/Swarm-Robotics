#include "Solution.hpp"

void Solution::finalize(std::vector<AgentInfo> agents)
{
    for(auto& agent : agents){
        auto& path = this->paths[agent.getId()];
        // Remove cat and work action
        std::vector<Action> pathActions;
        for (auto& a : path.actions){
            if (a.isWaitAction() && a.endVertex == agent.getGoal() && a.duration == TIME_AT_GOAL){
                path.cost = a.timestamp + a.duration;
                break;
            }
            pathActions.push_back(a);
        }
        path.actions = pathActions;

        // Remove agents current action
        path.actions.erase(path.actions.begin());

        Error::log(agent.getCurrentAction().toString() + " " + path.toString() + "\n");
    }
}