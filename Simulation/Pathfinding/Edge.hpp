#ifndef EDGE_HPP
#define EDGE_HPP

class Edge;

#include "Vertex.hpp"

class Edge {
public:
    Edge(std::shared_ptr<Vertex> startVertex, std::shared_ptr<Vertex> endVertex, float cost);
    std::shared_ptr<Vertex> getStartVertex();
    std::shared_ptr<Vertex> getEndVertex();
    float getCost();
    std::string toString();
private:
    float cost;
    std::shared_ptr<Vertex> startVertex;
    std::shared_ptr<Vertex> endVertex;  
};

#endif