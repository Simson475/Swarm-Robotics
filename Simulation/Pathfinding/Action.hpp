#ifndef ACTION_HPP
#define ACTION_HPP

#include "models/map/map_structure.hpp"
#include "point.hpp"
#include "Vertex.hpp"

class Action {
  public:
    Action() = default;
    Action(const Action&);
    Action(Action&&) noexcept;
    Action(int timestamp, std::shared_ptr<Vertex> startVertex, std::shared_ptr<Vertex> endVertex, float duration){
      this->timestamp = timestamp;
      this->startVertex = startVertex;
      this->endVertex = endVertex;
      this->duration = duration;
    }
    ~Action() = default;
    void operator=(const Action &a);
    int timestamp;
    std::shared_ptr<Vertex> startVertex;
    std::shared_ptr<Vertex> endVertex;
    float duration;
};

#endif