#include "Graph.hpp"

#define ROBOT_SPEED (0.1)

Graph::Graph(Map_Structure& map){
    // All vertices
    size_t pointCount = map.points.size();
    std::vector<std::shared_ptr<Vertex>> vertices{pointCount};
    for (Point point : map.points){
        int id = point.getId();
        vertices[id] = std::make_shared<Vertex>(id);//TODO params for constructor
    }
    this->vertices = vertices;

    // All edges
    size_t lineCount = map.lines.size();
    std::vector<std::vector<std::shared_ptr<Edge>>> edges{lineCount};
    for (Line line : map.lines){
        int a = line.Geta().getId();
        int b = line.Getb().getId();
        edges[a].push_back(std::make_shared<Edge>(Edge(
            vertices[a],
            vertices[b],
            line.GetDistance() / ROBOT_SPEED
        )));
    }
    this->edges = edges;
    
    // Add reference to edges in the vertices
    for (std::shared_ptr<Vertex> v : vertices){
        v->setEdges(edges[v->getId()]);
    }
}

float Graph::heuristicCost(std::shared_ptr<Vertex> from, std::shared_ptr<Vertex> to){
    if ( ! this->heuristicCosts.empty()){
        return this->heuristicCosts[from->getId()][to->getId()];
    }
    /* Compute heuristic costs since we dont have them computed yet */
    // Make the heuristicCosts a square matrix
    size_t size = this->vertices.size();
    this->heuristicCosts.resize(size);
    for (size_t i = 0; i < size; ++i){
        this->heuristicCosts[i].resize(size);
        for (size_t j = 0; j < size; ++j){
            this->heuristicCosts[i][j] = ((i == j) ? 0 : INFINITY);
        }
        for (std::shared_ptr<Edge> e : from->getEdges()){
            this->heuristicCosts[i][e->getEndVertex()->getId()] = e->getCost();
        }
    }
    for (size_t k = 0; k < size; ++k){
    for (size_t i = 0; i < size; ++i){
    for (size_t j = 0; j < size; ++j){
        float directCost = this->heuristicCosts[i][j];
        float indirectCost = this->heuristicCosts[i][k] + this->heuristicCosts[k][j];
        if (directCost > indirectCost)
            this->heuristicCosts[i][j] = indirectCost;
    }}}

    return this->heuristicCosts[from->getId()][to->getId()];
}