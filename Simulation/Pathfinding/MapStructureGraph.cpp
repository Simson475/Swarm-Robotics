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
    float robotSpeed = 0.061;
    for (Line line : map.lines){
        if (line.GetDistance() <= 0) { continue; }
        if ( ! (line.Geta().getId() < 4 || line.Getb().getId() < 4)){
            if ((std::abs(line.Geta().GetX() - line.Getb().GetX()) > 1) && (std::abs(line.Geta().GetY() - line.Getb().GetY()) > 1)
            ){
                continue;
            }
            if (line.GetDistance() / robotSpeed > 90) continue;
        }
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

    // // Prune transitive closure edges
    // for (auto& ve : edges){
    //     auto it = ve.begin();
    //     for (auto e : ve){
    //         for (auto e1 : e->getEndVertex()->getEdges()){
    //             for (auto e2 : e1->getEndVertex()->getEdges()){
    //                 if (e2->getEndVertex() == e->getStartVertex()){
    //                     // We found a transitive edge
    //                     ve.erase(it);
    //                     Error::log("We deleted an edge\n");
    //                 }
    //             }
    //         }
    //         it++;
    //     }
    // }
    
    // Add reference to edges in the vertices
    for (std::shared_ptr<Vertex> v : vertices){
        v->setEdges(edges[v->getId()]);
    }
}

std::vector<int> MapStructureGraph::getStations(){
    return this->stations;
}