#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include "Vertex.hpp"
#include "Edge.hpp"
#include <memory>
#include "Debugging.hpp"
#include <limits>
#include <queue>
#include <set>

class Graph {
public:
    Graph() = default;
    Graph(std::vector<std::shared_ptr<Vertex>> vertices);
    virtual ~Graph() = default;
    std::vector<std::shared_ptr<Vertex>> getVertices();
    float heuristicCost(std::shared_ptr<Vertex> from, std::shared_ptr<Vertex> to);
protected:
    std::vector<std::shared_ptr<Vertex>> vertices;
    std::vector<std::vector<std::shared_ptr<Edge>>> edges;
    std::vector<std::vector<float>> heuristicCosts;
    void reduceToTransitiveReduction();
};

#endif