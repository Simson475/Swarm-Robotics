#ifndef EDGE_HPP
#define EDGE_HPP

class Edge;

#include "Vertex.hpp"

class Edge {
public:
    Edge(Vertex StartVertex, Vertex EndVertex, float cost);
    Vertex& getStartVertex();
    Vertex& getEndVertex();
    float getCost();
private:
    float cost;
    Vertex& startVertex;
    Vertex& endVertex;  
};

#endif