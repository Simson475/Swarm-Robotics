#ifndef BACKWARDS_ACTION_PATH_AUX_HPP
#define BACKWARDS_ACTION_PATH_AUX_HPP

#include "Action.hpp"
#include "_path.hpp"
#include <memory>
#include "Debugging.hpp"
#include "GLOBALS.hpp"
#include "ActionPathAux.hpp"

class BackwardsActionPathAux{
public:
    BackwardsActionPathAux(Action action, float heuristic, std::shared_ptr<BackwardsActionPathAux> successor);
    BackwardsActionPathAux(const BackwardsActionPathAux &a);
    BackwardsActionPathAux(const ActionPathAux &a);
    Path getPath(float currentTime);

    Action action;
    float heuristic;
    std::shared_ptr<BackwardsActionPathAux> successor;
    void operator=(const BackwardsActionPathAux &a);
    std::string toString();
};

bool operator< (const BackwardsActionPathAux &a, const BackwardsActionPathAux &b);//Comparison function for priority queue
bool operator> (const BackwardsActionPathAux &a, const BackwardsActionPathAux &b);//Comparison function for priority queue

#endif