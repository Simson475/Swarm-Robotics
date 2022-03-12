#ifndef AGENT_HPP
#define AGENT_HPP
class Agent;

#include "_path.hpp"
#include "point.hpp"
#include <queue>
//#include "SingleThreadBotCBS.hpp"

class Agent {
  public:
    Path getPath();
    //void setBot(SingleThreadBotCBS*);
    //SingleThreadBotCBS* getBot();
    void createPath(std::vector<Point> plan);
    void recreatePath();
  private:
    Path path;
    std::vector<Point> plan;
    //SingleThreadBotCBS *bot;
};

#endif