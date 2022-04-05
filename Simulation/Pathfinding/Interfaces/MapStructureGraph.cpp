#include "MapStructureGraph.hpp"

MapStructureGraph::MapStructureGraph(Map_Structure& map){
    // All vertices
    size_t pointCount = map.points.size();
    std::vector<std::shared_ptr<Vertex>> vertices{pointCount};
    for (Point point : map.points){
        int id = point.getId();
        vertices[id] = std::make_shared<Vertex>(id);//TODO params for constructor
    }
    this->vertices = vertices;

    // All edges
    size_t lineCount = map.lines.size();
    std::vector<std::vector<std::shared_ptr<Edge>>> edges{lineCount};
    float robotSpeed = 0.1;
    for (Line line : map.lines){
        if (line.GetDistance() <= 0) { continue; }
        int a = line.Geta().getId();
        int b = line.Getb().getId();
        edges[a].push_back(std::make_shared<Edge>(Edge(
            vertices[a],
            vertices[b],
            line.GetDistance() / robotSpeed
        )));
    }
    this->edges = edges;
    
    // Add reference to edges in the vertices
    for (std::shared_ptr<Vertex> v : vertices){
        v->setEdges(edges[v->getId()]);
    }
}