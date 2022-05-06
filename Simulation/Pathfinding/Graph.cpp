#include "Graph.hpp"

Graph::Graph(std::vector<std::shared_ptr<Vertex>> vertices){
    this->vertices = vertices;
}

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
                if (directCost > indirectCost) this->heuristicCosts[i][j] = indirectCost;
            }
        }
    }

    return this->heuristicCosts[from->getId()][to->getId()];
}

/**
 * @brief Reduce this graph to the transitive reduction
 * NOTE: Currently does not reduce this->edges
 * 
 */
void Graph::reduceToTransitiveReduction(){
    std::queue<std::shared_ptr<Edge>> transitiveEdges;
    for (auto a : this->vertices){
        for (auto b : this->vertices){
            if (a == b) continue;
            auto ab = a->getEdge(b);
            if (ab == nullptr) continue;
            for (auto c : this->vertices){
                if (b == c) continue;
                auto bc = b->getEdge(c);
                auto ac = a->getEdge(c);
                // If ac, ab, and bc exists, delete the longest edge
                if (bc != nullptr && ac != nullptr){
                    auto acCost = ac->getCost();
                    auto abCost = ab->getCost();
                    auto bcCost = bc->getCost();
                    float maxCost = std::max(acCost, std::max(abCost, bcCost));
                    if (acCost == maxCost){
                        transitiveEdges.push(ac);
                    }
                    else if (abCost == maxCost){
                        transitiveEdges.push(ab);
                    }
                    else if (bcCost == maxCost){
                        transitiveEdges.push(bc);
                    }
                }
            }
        }
    }
    // Delete the transitive edges
    int size = transitiveEdges.size();
    for (int i = 0; i < size; ++i){
        auto e = transitiveEdges.front();
        transitiveEdges.pop();
        e->getStartVertex()->removeEdge(e);
    }
}