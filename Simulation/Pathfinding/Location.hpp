#ifndef LOCATION_HPP
#define LOCATION_HPP

#include "Vertex.hpp"
#include "Edge.hpp"

enum ELocationType{
    VERTEX_LOCATION,
    EDGE_LOCATION
};

class Location {
public:
    Location(){type=VERTEX_LOCATION;}
    Location(ELocationType, std::shared_ptr<Vertex>);
    Location(ELocationType, std::shared_ptr<Edge>);
    ELocationType type;
    std::shared_ptr<Edge> edge;
    std::shared_ptr<Vertex> vertex;
};

#endif