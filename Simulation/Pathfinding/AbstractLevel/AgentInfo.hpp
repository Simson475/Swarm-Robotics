#ifndef AGENT_INFO_HPP
#define AGENT_INFO_HPP

#include "Location.hpp"

class AgentInfo {
public:
    int getId();
    Location getLocation();
    Vertex getGoal();
private:
    int id;
    Location location;
    Vertex goal;
};

#endif