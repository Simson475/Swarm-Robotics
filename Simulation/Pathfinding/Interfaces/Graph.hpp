#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include "Vertex.hpp"
#include "Edge.hpp"
#include <memory>
#include "map_structure.hpp"

class Graph {
public:
    Graph(Map_Structure& map);
    std::vector<std::shared_ptr<Vertex>> getVertices();
    float heuristicCost(std::shared_ptr<Vertex> from, std::shared_ptr<Vertex> to);
    //void setVertices(std::vector<std::shared_ptr<Vertex>>);
    //std::vector<std::shared_ptr<Edge>> getEdges();
    //void setEdges(std::vector<std::shared_ptr<Edge>>);
protected:
    std::vector<std::shared_ptr<Vertex>> vertices;
    std::vector<std::vector<std::shared_ptr<Edge>>> edges;
    std::vector<std::vector<float>> heuristicCosts{};
};

#endif