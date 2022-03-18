#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include "Vertex.hpp"
#include "Edge.hpp"

class Graph {
public:
    std::vector<Vertex*> getVertices();
    void setVertices(std::vector<Vertex*>);
    std::vector<Edge*> getEdges();
    void setEdges(std::vector<Edge*>);
protected:
    std::vector<Vertex*> vertices;
    std::vector<Edge*> edges;
};

#endif