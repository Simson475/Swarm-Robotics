#ifndef AGENT_INFO_HPP
#define AGENT_INFO_HPP

#include "Location.hpp"
#include <memory>
#include "HighLevelCBS.hpp"

class AgentInfo {
public:
    AgentInfo() = default;
    AgentInfo(const AgentInfo&){};
    int getId();
    void setId(size_t id);
    std::shared_ptr<Location> getLocation();
    std::shared_ptr<Vertex> getGoal();
private:
    int id;
    std::shared_ptr<Location> location;
    std::shared_ptr<Vertex> goal;
};

#endif