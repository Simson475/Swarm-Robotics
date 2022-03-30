#ifndef CONFLICT_HPP
#define CONFLICT_HPP

class Conflict;

#include <vector>
#include "Location.hpp"
#include "Agent.hpp"
#include <memory>

class Conflict
{
public:
    //TODO can any of the constructors/destructors be pruned
    Conflict(std::vector<std::shared_ptr<Agent>> agents, int timeStart, int timeEnd, Location location);
    Conflict(std::vector<int> agents, int timeStart, int timeEnd, Location location);
    Conflict() = default;
    Conflict(Conflict*);
    Conflict(const Conflict&);
    Conflict(Conflict&&) noexcept;
    ~Conflict();

    std::vector<std::shared_ptr<Agent>> getAgents();
    int getTimeStart();
    int getTimeEnd();
    Location getLocation();
private:
    std::vector<std::shared_ptr<Agent>> agents;
    int timeStart;
    int timeEnd;
    Location location;
};

#endif