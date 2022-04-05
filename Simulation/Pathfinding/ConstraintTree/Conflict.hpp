#ifndef CONFLICT_HPP
#define CONFLICT_HPP

class Conflict;

#include <vector>
#include "Location.hpp"
#include <memory>

class Conflict
{
public:
    //TODO can any of the constructors/destructors be pruned
    Conflict(std::vector<int> agentIds, int timeStart, int timeEnd, Location location);
    Conflict() = default;
    Conflict(Conflict*);
    Conflict(const Conflict&);
    Conflict(Conflict&&);
    ~Conflict() = default;

    std::vector<int> getAgentIds();
    int getTimeStart();
    int getTimeEnd();
    Location getLocation();
private:
    std::vector<int> agentIds;
    int timeStart;
    int timeEnd;
    Location location;
};

#endif