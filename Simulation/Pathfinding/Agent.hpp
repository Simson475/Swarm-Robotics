#ifndef AGENT_HPP
#define AGENT_HPP
class Agent;

#include "Path.hpp"
#include "point.hpp"
#include <queue>
#include "TestController.hpp"
#include "Action.hpp"
#include "Location.hpp"
#include "AgentInfo.hpp"

class Agent {
  public:
    Agent(int id, TestController* controller);
    void setBot(TestController*);
    TestController* getBot();
    Location getLocation();
    int getId();
    Action getCurrentAction();
    std::shared_ptr<Vertex> getGoal();
    AgentInfo getAgentInfo();
  private:
    TestController *bot;
    int id;
    Action currentAction;
};

#endif