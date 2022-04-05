#ifndef AGENT_INFO_HPP
#define AGENT_INFO_HPP

#include "Location.hpp"
#include "Action.hpp"
#include "Vertex.hpp"
#include <memory>

class AgentInfo {
public:
    AgentInfo();
    AgentInfo(const AgentInfo& a);
    AgentInfo(AgentInfo* a);
    AgentInfo(AgentInfo&&);
    AgentInfo(int id, Action currentAction, std::shared_ptr<Vertex> destination);
    int getId();
    Action getCurrentAction();
    std::shared_ptr<Vertex> getGoal();
    int getTimeAtVertex(std::shared_ptr<Vertex> vertex);
    void operator=(const AgentInfo& other);
private:
    int id;
    Action currentAction;
    std::shared_ptr<Vertex> goal;
};

#endif