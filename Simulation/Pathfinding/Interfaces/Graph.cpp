#include "Graph.hpp"

std::vector<std::shared_ptr<Vertex>> Graph::getVertices(){
    return this->vertices;
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
            this->heuristicCosts[i][j] = ((i == j) ? 0 : std::numeric_limits<float>::infinity());
        }
        for (std::shared_ptr<Edge> e : this->vertices[i]->getEdges()){
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