#ifndef ACTION_PATH_AUX_HPP
#define ACTION_PATH_AUX_HPP

#include "Action.hpp"
#include "_path.hpp"
#include <memory>

class ActionPathAux{
public:
    ActionPathAux(Action action, int priority, std::shared_ptr<ActionPathAux> predecessor);
    Path getPath();

    Action action;
    int priority;
    std::shared_ptr<ActionPathAux> predecessor;
};

bool operator< (const ActionPathAux &a, const ActionPathAux &b);//Comparison function for priority queue

#endif