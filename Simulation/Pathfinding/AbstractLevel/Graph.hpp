#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include "Vertex.hpp"
#include "Edge.hpp"

class Graph {
public:
    Graph(std::vector<Vertex> vertices, std::vector<Edge> edges);
    std::vector<Vertex> getVertices();
    std::vector<Edge> getEdges();
protected:
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
};

#endif