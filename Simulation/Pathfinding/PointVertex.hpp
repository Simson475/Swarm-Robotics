#ifndef VERTEX_HPP
#define VERTEX_HPP

#include "Point.hpp"

class PointVertex : Vertex {
public:
    PointVertex(float x, float y)
    float getX() override;
    float getY() override;
protected:
    Point point;
};

#endif