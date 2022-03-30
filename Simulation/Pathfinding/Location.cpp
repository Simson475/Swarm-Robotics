#include "Location.hpp"

Location::Location(ELocationType type, std::shared_ptr<Vertex> v){
    this->type = type;
    this->vertex = v;
}

Location::Location(ELocationType type, std::shared_ptr<Edge> edge){
    this->type = type;
    this->edge = edge;
}