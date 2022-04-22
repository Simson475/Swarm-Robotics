#ifndef MAPSTRUCTURE_GRAPH_HPP
#define MAPSTRUCTURE_GRAPH_HPP

#include <vector>
#include "Vertex.hpp"
#include "Edge.hpp"
#include <memory>
#include "map_structure.hpp"
#include "Graph.hpp"
#include "Debugging.hpp"
#include "DEFINITIONS.hpp"

class MapStructureGraph : public Graph{
public:
    virtual ~MapStructureGraph() = default;
    MapStructureGraph(Map_Structure& map);
    std::vector<int> getStations();
private:
    std::vector<int> stations;
};

#endif