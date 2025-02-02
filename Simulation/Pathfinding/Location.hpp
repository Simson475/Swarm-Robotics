#ifndef LOCATION_HPP
#define LOCATION_HPP

#include "Vertex.hpp"
#include "Edge.hpp"
#include <iostream>

enum ELocationType{
    VERTEX_LOCATION,
    EDGE_LOCATION
};

class Location {
public:
    Location(){type=VERTEX_LOCATION;}
    Location(std::shared_ptr<Vertex>);
    Location(std::shared_ptr<Edge>);
    ELocationType type;
    std::shared_ptr<Edge> edge;
    std::shared_ptr<Vertex> vertex;
    std::string toString() const;
    bool operator==(const Location& location);
    bool operator==(std::shared_ptr<Vertex> vertex);
    bool operator==(std::shared_ptr<Edge> edge);
};

#endif