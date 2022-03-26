#include "Action.hpp"

Action::Action(const Action& a){
    timestamp = a.timestamp;
    startVertex = a.startVertex;
    endVertex = a.endVertex;
    duration = a.duration;
}

Action::Action(int timestamp, std::shared_ptr<Vertex> startVertex, std::shared_ptr<Vertex> endVertex, float duration){
    this->timestamp = timestamp;
    this->startVertex = startVertex;
    this->endVertex = endVertex;
    this->duration = duration;
}

Action::Action(Action&& a){
    timestamp = a.timestamp;
    startVertex = a.startVertex;
    endVertex = a.endVertex;
    duration = a.duration;
}

void Action::operator=(const Action &a){
    timestamp = a.timestamp;
    startVertex = a.startVertex;
    endVertex = a.endVertex;
    duration = a.duration;
}