#ifndef VERTEX_HPP
#define VERTEX_HPP

class Vertex;

#include <vector>
#include <memory>
#include "Edge.hpp"
#include "Debugging.hpp"
#include <iostream>

class Vertex {
public:
    Vertex(int id);
    std::vector<std::shared_ptr<Edge>> getEdges();
    std::shared_ptr<Edge> getEdge(std::shared_ptr<Vertex>);
    void setEdges(std::vector<std::shared_ptr<Edge>> edges);
    void addEdge(std::shared_ptr<Edge> edge);
    void removeEdge(std::shared_ptr<Edge> edge);
    int getId();
    std::string toString();
protected:
    std::vector<std::shared_ptr<Edge>> edges;
    int id;
};

#endif