#include "Edge.hpp"

Vertex& Edge::getStartVertex(){
    return startVertex;
}
Vertex& Edge::getEndVertex(){
    return endVertex;
}
float Edge::getCost(){
    return cost;//TODO figure out how we can use this as an interface instead or if we need an adapter
}