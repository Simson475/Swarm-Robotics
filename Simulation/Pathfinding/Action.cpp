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

bool Action::operator==(const Action &a){
    return timestamp == a.timestamp
        && startVertex == a.startVertex
        && endVertex == a.endVertex
        && duration == a.duration;
}

bool Action::isWaitAction(){
    return startVertex->getId() == endVertex->getId();
}

Location Action::getLocation(){
    if (isWaitAction()){
        return Location(ELocationType::VERTEX_LOCATION, startVertex);
    }
    for (auto e : startVertex->getEdges()){
        if (e->getEndVertex()->getId() == endVertex->getId())
        return Location(ELocationType::EDGE_LOCATION, e);
    }
    exit(1);
}