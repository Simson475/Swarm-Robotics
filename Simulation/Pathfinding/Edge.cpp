#include "Edge.hpp"

Edge::Edge(std::shared_ptr<Vertex> startVertex, std::shared_ptr<Vertex> endVertex, float cost){
    this->startVertex = startVertex;
    this->endVertex = endVertex;
    this->cost = cost;
}

std::shared_ptr<Vertex> Edge::getStartVertex(){
    return startVertex;
}
std::shared_ptr<Vertex> Edge::getEndVertex(){
    return endVertex;
}
float Edge::getCost(){
    return cost;
}

std::string Edge::toString(){
    return "std::make_shared<Edge>(v" + std::to_string( startVertex->getId()) + ", v" + std::to_string( endVertex->getId()) +", " +std::to_string( cost) +")";
    return "e["+(std::to_string(startVertex->getId()))+"," + (std::to_string(endVertex->getId())) +"]";
}