#include "Vertex.hpp"

Vertex::Vertex(int id){
    this->id = id;
}
std::vector<std::shared_ptr<Edge>> Vertex::getEdges(){
    return this->edges;
}
void Vertex::setEdges(std::vector<std::shared_ptr<Edge>> edges){
    this->edges = edges;
}
int Vertex::getId(){
    return this->id;
}

std::string Vertex::toString(){
    return "v" + std::to_string(id);
}