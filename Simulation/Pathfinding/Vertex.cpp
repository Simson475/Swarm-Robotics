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
    return "v" + std::to_string(id);
}

void Vertex::addEdge(std::shared_ptr<Edge> edge){
    this->edges.push_back(edge);
}
void Vertex::removeEdge(std::shared_ptr<Edge> edge){
    for(auto it = this->edges.begin(); it < this->edges.end(); it++){
        if (*(it.base()) == edge){
            this->edges.erase(it);
        }
    }
}