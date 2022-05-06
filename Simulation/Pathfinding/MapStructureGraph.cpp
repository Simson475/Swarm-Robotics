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
        #ifdef DEBUG_LOGS_ON
        Error::log(vertices[id]->toString() + "(" + std::to_string(point.GetX()) + "," + std::to_string(point.GetY()) + ")\n");
        #endif
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
        #ifdef DEBUG_LOGS_ON
        Error::log(edge->toString() + " " + std::to_string(edge->getCost()) + "\n");
        #endif
    }
    this->edges = edges;
    
    // Add reference to edges in the vertices
    for (std::shared_ptr<Vertex> v : vertices){
        v->setEdges(edges[v->getId()]);
    }

    this->reduceToTransitiveReduction();

    #ifdef DEBUG_LOGS_ON
    for (auto v : this->vertices){
        for (auto e : v->getEdges()){
            Error::log(e->toString() + " " + std::to_string(e->getCost()) + "\n");
        }
    }
    #endif
}

std::vector<int> MapStructureGraph::getStations(){
    return this->stations;
}