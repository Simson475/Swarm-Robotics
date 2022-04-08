#include "GraphTests.hpp"

void GraphTests::heuristic_costs_are_correct(){
    // Arrange
    /**
     * We will construct a triangle-ish shaped graph
     * 1
     * | \ 
     * 2--3-----------4         5
    */
    auto v1 = std::make_shared<Vertex>(0);
    auto v2 = std::make_shared<Vertex>(1);
    auto v3 = std::make_shared<Vertex>(2);
    auto v4 = std::make_shared<Vertex>(3);
    auto v5 = std::make_shared<Vertex>(4);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v2, 1),
        std::make_shared<Edge>(v1, v3, 3),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v1, 1),
        std::make_shared<Edge>(v2, v3, 1),
    };
    std::vector<std::shared_ptr<Edge>> v3edges = {
        std::make_shared<Edge>(v3, v1, 3),
        std::make_shared<Edge>(v3, v2, 1),
        std::make_shared<Edge>(v3, v4, 100),
    };
    
    v1->setEdges(v1edges);
    v2->setEdges(v2edges);
    v3->setEdges(v3edges);
    std::vector<std::shared_ptr<Vertex>> vertices = {v1, v2, v3, v4, v5};
    Graph g = Graph(vertices);

    // Act
    float v1v1Cost = g.heuristicCost(v1, v1);
    float v1v3Cost = g.heuristicCost(v1, v3);
    float v1v4Cost = g.heuristicCost(v1, v4);
    float v1v5Cost = g.heuristicCost(v1, v5);

    // Assert
    assert(v1v1Cost == 0);
    assert(v1v3Cost == 2);
    assert(v1v4Cost == 102);
    assert(v1v5Cost == std::numeric_limits<float>::infinity());
}