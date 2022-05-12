#include "Action.hpp"

Action::Action(const Action& a){
    timestamp = a.timestamp;
    startVertex = a.startVertex;
    endVertex = a.endVertex;
    duration = a.duration;
}

Action::Action(float timestamp, std::shared_ptr<Vertex> startVertex, std::shared_ptr<Vertex> endVertex, float duration){
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

bool Action::operator==(const Action &a) const{
    return timestamp == a.timestamp
        && startVertex == a.startVertex
        && endVertex == a.endVertex
        && duration == a.duration;
}

bool Action::operator!=(const Action &a) const{
    return ! this->operator==(a);
}

bool Action::isWaitAction(){
    return startVertex->getId() == endVertex->getId();
}

Location Action::getLocation(){
    if (isWaitAction()){
        return Location(startVertex);
    }
    for (auto e : startVertex->getEdges()){
        if (e->getEndVertex()->getId() == endVertex->getId())
        return Location(e);
    }
    exit(1);
}

std::string Action::toString(){
    return "{v" + std::to_string(this->startVertex->getId()) + "->" + "v" + std::to_string(this->endVertex->getId()) + " t[" + std::to_string(this->timestamp) + "," + std::to_string(this->timestamp + this->duration) + "]}";
}

void Action::sync(float desyncOffset){
    this->timestamp += desyncOffset;
    if (this->isWaitAction()){
        this->duration = std::max(0.0f, this->duration - desyncOffset);
    }
}