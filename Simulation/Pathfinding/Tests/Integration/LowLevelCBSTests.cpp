#include "LowLevelCBSTests.hpp"

void LowLevelCBSTests::pathfinding_is_correct(){
    // Arrange
    /**
     * We will construct a + shaped graph
     *     0
     *   / | \
     * 1---2---3
     *   \ | /
     *     4
     * straight lines are length 1
     * diagonal lines are length 1.4
    */
    auto v0 = std::make_shared<Vertex>(0);
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    auto v3 = std::make_shared<Vertex>(3);
    auto v4 = std::make_shared<Vertex>(4);
    std::vector<std::shared_ptr<Edge>> v0edges = {
        std::make_shared<Edge>(v0, v1, 1.4),
        std::make_shared<Edge>(v0, v2, 1),
        std::make_shared<Edge>(v0, v3, 1.4),
    };
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v0, 1.4),
        std::make_shared<Edge>(v1, v2, 1),
        std::make_shared<Edge>(v1, v4, 1.4),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v0, 1),
        std::make_shared<Edge>(v2, v1, 1),
        std::make_shared<Edge>(v2, v3, 1),
        std::make_shared<Edge>(v2, v4, 1),
    };
    std::vector<std::shared_ptr<Edge>> v3edges = {
        std::make_shared<Edge>(v3, v0, 1.4),
        std::make_shared<Edge>(v3, v2, 1),
        std::make_shared<Edge>(v3, v4, 1.4),
    };
    std::vector<std::shared_ptr<Edge>> v4edges = {
        std::make_shared<Edge>(v4, v1, 1.4),
        std::make_shared<Edge>(v4, v2, 1),
        std::make_shared<Edge>(v4, v3, 1.4),
    };
    
    v0->setEdges(v0edges);
    v1->setEdges(v1edges);
    v2->setEdges(v2edges);
    v3->setEdges(v3edges);
    v4->setEdges(v4edges);
    std::vector<std::shared_ptr<Vertex>> vertices = {v0, v1, v2, v3, v4};
    auto g = std::make_shared<Graph>(vertices);
    //AgentInfo(id, action, dest)
    std::vector<AgentInfo> agents = {
        AgentInfo(0, Action(0, v0, v0, 0), v4), // Starting in v0, going to v4
    };

    //Act
    Path path = LowLevelCBS::get_instance().getIndividualPath(g, agents[0], std::vector<Constraint>{});

    //Assert
    //Assert the correct path is output
    assert(path.actions[0].endVertex == v0);
    assert(path.actions[1].endVertex == v2);
    assert(path.actions[2].endVertex == v4);
    assert(path.cost == 2);
    assert(LowLevelCBS::get_instance().iterations == 3);// 2 from v0 to v4 and 1 from the initial action
}