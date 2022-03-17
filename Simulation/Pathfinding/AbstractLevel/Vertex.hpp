#ifndef VERTEX_HPP
#define VERTEX_HPP

class Vertex;

#include <vector>
#include "Edge.hpp"

class Vertex {
public:
    Vertex(float x, float y);
    float getX();
    float getY();
    std::vector<Edge> getEdges();
    void setEdges(std::vector<Edge> edges);
protected:
    float x;
    float y;
    std::vector<Edge> edges;
};

#endif