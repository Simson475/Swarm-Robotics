#ifndef ACTION_HPP
#define ACTION_HPP

#include "Vertex.hpp"
#include "Location.hpp"

class Action {
  public:
    Action() = default;
    Action(Action*);
    Action(const Action&);
    Action(Action&&);
    Action(float timestamp, std::shared_ptr<Vertex> startVertex, std::shared_ptr<Vertex> endVertex, float duration);
    ~Action() = default;
    void operator=(const Action &a);
    bool operator==(const Action &a) const;
    bool operator!=(const Action &a) const;
    float timestamp;
    std::shared_ptr<Vertex> startVertex;
    std::shared_ptr<Vertex> endVertex;
    float duration;
    bool isWaitAction();
    Location getLocation();
    std::string toString();
    void sync(float desyncOffset);
};

#endif