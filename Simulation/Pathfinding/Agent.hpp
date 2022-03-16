#ifndef AGENT_HPP
#define AGENT_HPP
class Agent;

#include "_path.hpp"
#include "point.hpp"
#include <queue>
//#include "SingleThreadBotCBS.hpp"
#include "TestController.hpp"

class Agent {
  public:
    Path getPath();
    void setBot(TestController*);
    TestController* getBot();
    void createPath(std::vector<Point> plan);
    //void recreatePath();
  private:
    Path path;
    std::vector<Point> plan;
    TestController *bot;
};

#endif