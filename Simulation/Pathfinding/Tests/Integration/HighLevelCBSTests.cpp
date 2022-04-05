#include "HighLevelCBSTests.hpp"

void HighLevelCBSTests::it_gets_a_path_that_has_no_conflicts(){
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
        std::make_shared<Edge>(v0, v1, 140),
        std::make_shared<Edge>(v0, v2, 100),
        std::make_shared<Edge>(v0, v3, 150),
    };
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v0, 140),
        std::make_shared<Edge>(v1, v2, 100),
        std::make_shared<Edge>(v1, v4, 140),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v0, 100),
        std::make_shared<Edge>(v2, v1, 100),
        std::make_shared<Edge>(v2, v3, 100),
        std::make_shared<Edge>(v2, v4, 100),
    };
    std::vector<std::shared_ptr<Edge>> v3edges = {
        std::make_shared<Edge>(v3, v0, 150),
        std::make_shared<Edge>(v3, v2, 100),
        std::make_shared<Edge>(v3, v4, 140),
    };
    std::vector<std::shared_ptr<Edge>> v4edges = {
        std::make_shared<Edge>(v4, v1, 140),
        std::make_shared<Edge>(v4, v2, 100),
        std::make_shared<Edge>(v4, v3, 140),
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
        AgentInfo(1, Action(0, v1, v1, 0), v3), // Starting in v1, going to v3
    };
    // Act
    Solution solution = HighLevelCBS::get_instance().findSolution(g, agents, LowLevelCBS::get_instance());

    // Assert
    assert(solution.paths.size() == 2);
    assert(solution.paths[0].cost == 221 || solution.paths[0].cost == 200);
    assert(solution.paths[1].cost == 221 || solution.paths[1].cost == 200);
    assert(solution.paths[0].cost != solution.paths[1].cost);
}

void HighLevelCBSTests::it_can_find_a_solution_if_agents_have_same_goal(){
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
        std::make_shared<Edge>(v0, v1, 140),
        std::make_shared<Edge>(v0, v2, 100),
        std::make_shared<Edge>(v0, v3, 150),
    };
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v0, 140),
        std::make_shared<Edge>(v1, v2, 100),
        std::make_shared<Edge>(v1, v4, 140),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v0, 100),
        std::make_shared<Edge>(v2, v1, 100),
        std::make_shared<Edge>(v2, v3, 100),
        std::make_shared<Edge>(v2, v4, 100),
    };
    std::vector<std::shared_ptr<Edge>> v3edges = {
        std::make_shared<Edge>(v3, v0, 150),
        std::make_shared<Edge>(v3, v2, 100),
        std::make_shared<Edge>(v3, v4, 140),
    };
    std::vector<std::shared_ptr<Edge>> v4edges = {
        std::make_shared<Edge>(v4, v1, 140),
        std::make_shared<Edge>(v4, v2, 100),
        std::make_shared<Edge>(v4, v3, 140),
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
        AgentInfo(1, Action(0, v1, v1, 0), v4), // Starting in v1, going to v4
    };
    // Act
    Solution solution = HighLevelCBS::get_instance().findSolution(g, agents, LowLevelCBS::get_instance());

    // Assert
    assert(solution.paths.size() == 2);
    std::cout << "Path0 cost: " << solution.paths[0].cost << "\n";
    std::cout << "Path1 cost: " << solution.paths[1].cost << "\n";
    assert(solution.paths[0].cost == 761);
    assert(solution.paths[1].cost == 440);
}