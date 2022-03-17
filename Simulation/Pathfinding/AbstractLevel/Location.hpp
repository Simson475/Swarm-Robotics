#ifndef LOCATION_HPP
#define LOCATION_HPP

#include "Vertex.hpp"
#include "Edge.hpp"

enum ELocationType{
    VERTEX_LOCATION,
    EDGE_LOCATION
};

union ULocationUnion{
    Vertex vertex;
    Edge edge;
};

class Location {
public:
    ELocationType type;
    ULocationUnion location;
};

#endif