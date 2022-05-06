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
    ActionPathAux(Action action, float heuristic, std::shared_ptr<ActionPathAux> predecessor, bool hasWorked = false);
    ActionPathAux(const ActionPathAux &a);
    Path getPath() const;
    Path getPathWithoutCAT() const;

    Action action;
    float heuristic;
    std::shared_ptr<ActionPathAux> predecessor;
    void operator=(const ActionPathAux &a);
    std::string toString();
    bool hasWorked;
};

bool operator< (const ActionPathAux &a, const ActionPathAux &b);//Comparison function for priority queue
bool operator> (const ActionPathAux &a, const ActionPathAux &b);//Comparison function for priority queue

#endif