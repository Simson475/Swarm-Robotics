#ifndef CONFLICT_HPP
#define CONFLICT_HPP

#include <vector>
#include "Location.hpp"

class Conflict
{
public:
    //TODO can any of the constructors/destructors be pruned
    Conflict() = default;
    Conflict(Conflict*);
    Conflict(const Conflict&);
    Conflict(Conflict&&) noexcept;
    ~Conflict();

    std::vector<AgentInfo> getAgents();
    int getTimeStart;
    int getTimeEnd;
    Location location;
private:
    std::vector<AgentInfo> agents;
    int timeStart;
    int timeEnd;
    Location location;
};

#endif