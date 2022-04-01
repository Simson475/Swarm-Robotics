#ifndef ACTION_PATH_AUX_HPP
#define ACTION_PATH_AUX_HPP

#include "Action.hpp"
#include "_path.hpp"
#include <memory>

class ActionPathAux{
public:
    ActionPathAux(Action action, float priority, std::shared_ptr<ActionPathAux> predecessor);
    ActionPathAux(const ActionPathAux &a);
    Path getPath();

    Action action;
    float priority;
    std::shared_ptr<ActionPathAux> predecessor;
    void operator=(const ActionPathAux &a);
};

bool operator< (const ActionPathAux &a, const ActionPathAux &b);//Comparison function for priority queue

#endif