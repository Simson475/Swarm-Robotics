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
    ULocationUnion() { /* Do absolutely nothing */};
    ~ULocationUnion() { /* Do nothing idk */};
};

class Location {
public:
    Location(){type=VERTEX_LOCATION;}
    ELocationType type;
    ULocationUnion location;
};

#endif