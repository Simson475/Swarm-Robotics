#include "MapStructureGraph.hpp"

MapStructureGraph::MapStructureGraph(Map_Structure& map){
    // All vertices
    size_t pointCount = map.points.size();
    std::vector<std::shared_ptr<Vertex>> vertices{pointCount};
    for (Point point : map.points){
        int id = point.getId();
        vertices[id] = std::make_shared<Vertex>(id);

        if (point.getType() == 2){
            stations.push_back(id);
        }
        
        Error::log(vertices[id]->toString() /*+ "(" + std::to_string(point.GetX()) + "," + std::to_string(point.GetY()) + ")*/+"\n");
        
    }
    this->vertices = vertices;

    // All edges
    size_t lineCount = map.lines.size();
    std::vector<std::vector<std::shared_ptr<Edge>>> edges{lineCount};
    float robotSpeed = 0.061; //An approximation of the actual robot speed

    for (Line line : map.lines){
        //this only works if robots are spawned at the top or bottom of the map
        if (line.GetDistance() <= 0) { continue; }
        int a = line.Geta().getId();
        int b = line.Getb().getId();
        auto edge = std::make_shared<Edge>(Edge(
            vertices[a],
            vertices[b],
            line.GetDistance() / robotSpeed
        ));
        edges[a].push_back(edge);
    }

    this->edges = edges;
    
    // Add reference to edges in the vertices
    for (std::shared_ptr<Vertex> v : vertices){
        v->setEdges(edges[v->getId()]);
    }

    this->reduceToTransitiveReduction();

        // Vertices
        Error::log("std::vector<std::shared_ptr<Vertex>> vertices = {\n");
        for(auto vertex : this->vertices){
            Error::log( "v"+std::to_string(vertex->getId())+",");
        }
        Error::log("};\n");

        // Edges
        for(auto vertex : this->vertices){
            Error::log("std::vector<std::shared_ptr<Edge>> v"+std::to_string(vertex->getId())+"edges = {");
            for (auto edge : vertex->getEdges()){
                Error::log(edge->toString() + ",");
            }
            Error::log("};\n");
            Error::log("v"+std::to_string(vertex->getId())+"->setEdges(v"+std::to_string(vertex->getId())+"edges);\n");
        }
}

std::vector<int> MapStructureGraph::getStations(){
    return this->stations;
}