#ifndef ACTION_HPP
#define ACTION_HPP

#include "models/map/map_structure.hpp"
#include "point.hpp"
#include "Vertex.hpp"
#include "Location.hpp"

class Action {
  public:
    Action() = default;
    Action(Action*);
    Action(const Action&);
    Action(Action&&);
    Action(int timestamp, std::shared_ptr<Vertex> startVertex, std::shared_ptr<Vertex> endVertex, float duration);
    ~Action() = default;
    void operator=(const Action &a);
    bool operator==(const Action &a);
    int timestamp;
    std::shared_ptr<Vertex> startVertex;
    std::shared_ptr<Vertex> endVertex;
    float duration;
    bool isWaitAction();
    Location getLocation();
};

#endif