#ifndef VERTEX_HPP
#define VERTEX_HPP

class Vertex;

#include <vector>
#include <memory>
#include "Edge.hpp"

class Vertex {
public:
    std::vector<std::shared_ptr<Edge>> getEdges();
    void setEdges(std::vector<std::shared_ptr<Edge>> edges);
    int getId();
protected:
    float x;
    float y;
    std::vector<std::shared_ptr<Edge>> edges;
    int id;
};

#endif