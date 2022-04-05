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
    Conflict(std::vector<int> agentIds, float timeStart, float timeEnd, Location location);
    Conflict() = default;
    Conflict(Conflict*);
    Conflict(const Conflict&);
    Conflict(Conflict&&);
    ~Conflict() = default;

    std::vector<int> getAgentIds();
    float getTimeStart();
    float getTimeEnd();
    Location getLocation();
    std::string toString();
private:
    std::vector<int> agentIds;
    float timeStart;
    float timeEnd;
    Location location;
};

#endif