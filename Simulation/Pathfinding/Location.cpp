#include "Location.hpp"

Location::Location(std::shared_ptr<Vertex> v){
    this->type = ELocationType::VERTEX_LOCATION;
    this->vertex = v;
}

Location::Location(std::shared_ptr<Edge> edge){
    this->type = ELocationType::EDGE_LOCATION;
    this->edge = edge;
}

std::string Location::toString() const {
    return (type == ELocationType::EDGE_LOCATION) ? edge->toString() : vertex->toString();
}

bool Location::operator==(const Location& location){
    return this->toString() == location.toString();//TODO maybe change
}
bool Location::operator==(std::shared_ptr<Vertex> vertex){
    return this->type == ELocationType::VERTEX_LOCATION && this->vertex == vertex;
}
bool Location::operator==(std::shared_ptr<Edge> edge){
    return this->type == ELocationType::EDGE_LOCATION && this->edge == edge;
}