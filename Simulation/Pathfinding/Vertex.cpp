#include "Vertex.hpp"

Vertex::Vertex(int id){
    this->id = id;
}
std::vector<std::shared_ptr<Edge>> Vertex::getEdges(){
    return this->edges;
}
std::shared_ptr<Edge> Vertex::getEdge(std::shared_ptr<Vertex> vertex){
    for (auto e : getEdges()){
        if (e->getEndVertex() == vertex) return e;
    }
    return nullptr;
}
void Vertex::setEdges(std::vector<std::shared_ptr<Edge>> edges){
    this->edges = edges;
}
int Vertex::getId(){
    return this->id;
}

std::string Vertex::toString(){
    return "auto v" + std::to_string(id) + " = std::make_shared<Vertex>(" + std::to_string(id) + ");";
    return "v" + std::to_string(id);
}