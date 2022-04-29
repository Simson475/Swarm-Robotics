#ifndef ACTION_PATH_AUX_HPP
#define ACTION_PATH_AUX_HPP

#include "Action.hpp"
#include "Path.hpp"
#include <memory>
#include "Debugging.hpp"
#include "GLOBALS.hpp"
#include <cassert>

class ActionPathAux{
public:
    ActionPathAux(Action action, float heuristic, std::shared_ptr<ActionPathAux> predecessor);
    ActionPathAux(const ActionPathAux &a);
    Path getPath() const;
    Path getPath(const Action& lastAction) const;

    Action action;
    float heuristic;
    std::shared_ptr<ActionPathAux> predecessor;
    void operator=(const ActionPathAux &a);
    std::string toString();
};

bool operator< (const ActionPathAux &a, const ActionPathAux &b);//Comparison function for priority queue
bool operator> (const ActionPathAux &a, const ActionPathAux &b);//Comparison function for priority queue

#endif