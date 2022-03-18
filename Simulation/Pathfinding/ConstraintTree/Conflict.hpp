#ifndef CONFLICT_HPP
#define CONFLICT_HPP

#include <vector>
#include "Location.hpp"
#include "AgentInfo.hpp"
#include <memory>

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
    int getTimeStart();
    int getTimeEnd();
    Location getLocation();
private:
    std::vector<AgentInfo> agents;
    int timeStart;
    int timeEnd;
    std::shared_ptr<Location> location;
};

#endif