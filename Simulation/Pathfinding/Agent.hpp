#ifndef AGENT_HPP
#define AGENT_HPP
class Agent;

#include "_path.hpp"
#include "point.hpp"
#include <queue>
#include "TestController.hpp"
#include "Action.hpp"
#include "Location.hpp"

class Agent {
  public:
    Agent(int id, TestController* controller);
    Path getPath();
    void setBot(TestController*);
    TestController* getBot();
    void createPath(std::vector<Point> plan);
    Location getLocation();
    int getId();
    Action getCurrentAction();
    std::shared_ptr<Vertex> getGoal();
    int getTimeAtVertex(std::shared_ptr<Vertex> vertex);
    //void recreatePath();
  private:
    Path path;
    std::vector<Point> plan;
    TestController *bot;
    int id;
    Action currentAction;
};

#endif