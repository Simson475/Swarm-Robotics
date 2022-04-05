#ifndef AGENT_HPP
#define AGENT_HPP
class Agent;

#include "_path.hpp"
#include "point.hpp"
#include <queue>
#include "TestController.hpp"
#include "Action.hpp"
#include "Location.hpp"
#include "AgentInfo.hpp"

class Agent {
  public:
    Agent(int id, TestController* controller);
    Path getPath();
    void setBot(TestController*);
    TestController* getBot();
    Location getLocation();
    int getId();
    Action getCurrentAction();
    std::shared_ptr<Vertex> getGoal();
    AgentInfo getAgentInfo();
  private:
    Path path;
    std::vector<Point> plan;
    TestController *bot;
    int id;
    Action currentAction;
};

#endif