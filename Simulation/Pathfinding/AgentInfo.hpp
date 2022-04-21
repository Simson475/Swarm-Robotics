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
    int getId() const;
    Action getCurrentAction() const;
    std::shared_ptr<Vertex> getGoal() const;
    void operator=(const AgentInfo& other);
private:
    int id;
    Action currentAction;
    std::shared_ptr<Vertex> goal;
};

#endif