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
    assert(solution.paths[0].cost == 521 || solution.paths[0].cost == 500);
    assert(solution.paths[1].cost == 521 || solution.paths[1].cost == 500);
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
     * straight lines are length 100
     * diagonal lines are length 140 (except 0-3 which is 150)
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

void HighLevelCBSTests::it_can_find_a_solution_in_a_big_graph(){
    // Arrange
    /**
     * We will construct the following graph
     * 0-1-2-3
     * | | | |
     * 4-5-6-7
     * | | | |
     * 8-9-a-b
     * - lines are 100 length
     * | are 400 length
    */
    auto v0 = std::make_shared<Vertex>(0x0);
    auto v1 = std::make_shared<Vertex>(0x1);
    auto v2 = std::make_shared<Vertex>(0x2);
    auto v3 = std::make_shared<Vertex>(0x3);
    auto v4 = std::make_shared<Vertex>(0x4);
    auto v5 = std::make_shared<Vertex>(0x5);
    auto v6 = std::make_shared<Vertex>(0x6);
    auto v7 = std::make_shared<Vertex>(0x7);
    auto v8 = std::make_shared<Vertex>(0x8);
    auto v9 = std::make_shared<Vertex>(0x9);
    auto va = std::make_shared<Vertex>(0xa);
    auto vb = std::make_shared<Vertex>(0xb);
    float l = 400, s = 100;
    std::vector<std::shared_ptr<Edge>> v0edges = {
        std::make_shared<Edge>(v0, v1, s),
        std::make_shared<Edge>(v0, v4, l),
    };
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v0, s),
        std::make_shared<Edge>(v1, v2, s),
        std::make_shared<Edge>(v1, v5, l),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v1, s),
        std::make_shared<Edge>(v2, v3, s),
        std::make_shared<Edge>(v2, v6, l),
    };
    std::vector<std::shared_ptr<Edge>> v3edges = {
        std::make_shared<Edge>(v3, v2, s),
        std::make_shared<Edge>(v3, v7, l),
    };

    std::vector<std::shared_ptr<Edge>> v4edges = {
        std::make_shared<Edge>(v4, v0, l),
        std::make_shared<Edge>(v4, v5, s),
        std::make_shared<Edge>(v4, v8, l),
    };
    std::vector<std::shared_ptr<Edge>> v5edges = {
        std::make_shared<Edge>(v5, v1, l),
        std::make_shared<Edge>(v5, v4, s),
        std::make_shared<Edge>(v5, v6, s),
        std::make_shared<Edge>(v5, v9, l),
    };
    std::vector<std::shared_ptr<Edge>> v6edges = {
        std::make_shared<Edge>(v6, v2, l),
        std::make_shared<Edge>(v6, v5, s),
        std::make_shared<Edge>(v6, v7, s),
        std::make_shared<Edge>(v6, va, l),
    };
    std::vector<std::shared_ptr<Edge>> v7edges = {
        std::make_shared<Edge>(v7, v3, l),
        std::make_shared<Edge>(v7, v6, s),
        std::make_shared<Edge>(v7, vb, l),
    };

    std::vector<std::shared_ptr<Edge>> v8edges = {
        std::make_shared<Edge>(v8, v9, s),
        std::make_shared<Edge>(v8, v4, l),
    };
    std::vector<std::shared_ptr<Edge>> v9edges = {
        std::make_shared<Edge>(v9, v8, s),
        std::make_shared<Edge>(v9, va, s),
        std::make_shared<Edge>(v9, v5, l),
    };
    std::vector<std::shared_ptr<Edge>> vaedges = {
        std::make_shared<Edge>(va, v9, s),
        std::make_shared<Edge>(va, vb, s),
        std::make_shared<Edge>(va, v6, l),
    };
    std::vector<std::shared_ptr<Edge>> vbedges = {
        std::make_shared<Edge>(vb, va, s),
        std::make_shared<Edge>(vb, v7, l),
    };
    
    v0->setEdges(v0edges);
    v1->setEdges(v1edges);
    v2->setEdges(v2edges);
    v3->setEdges(v3edges);
    v4->setEdges(v4edges);
    v5->setEdges(v5edges);
    v6->setEdges(v6edges);
    v7->setEdges(v7edges);
    v8->setEdges(v8edges);
    v9->setEdges(v9edges);
    va->setEdges(vaedges);
    vb->setEdges(vbedges);
    std::vector<std::shared_ptr<Vertex>> vertices = {v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, va, vb};
    auto g = std::make_shared<Graph>(vertices);
    //AgentInfo(id, action, dest)
    std::vector<AgentInfo> agents = {
        AgentInfo(0, Action(0, v0, v0, 0), vb), // Starting in v0, going to vb
        AgentInfo(1, Action(0, v1, v1, 0), va),
        AgentInfo(2, Action(0, v2, v2, 0), v9),
        AgentInfo(3, Action(0, v3, v3, 0), v8),
        AgentInfo(4, Action(0, vb, vb, 0), v0),
        AgentInfo(5, Action(0, v8, v8, 0), v3), // Solvable
        AgentInfo(6, Action(0, v9, v9, 0), v2), // No longer solvable
        // AgentInfo(7, Action(0, va, va, 0), v1),
    };
    // Act
    Solution solution = HighLevelCBS::get_instance().findSolution(g, agents, LowLevelCBS::get_instance());

    // Assert
    //assert(solution.paths.size() == 2);
    std::cout << "Path0 cost: " << solution.paths[0].cost << "\n";
    std::cout << "Path1 cost: " << solution.paths[1].cost << "\n";
    //assert(solution.paths[0].cost == 761);
    //assert(solution.paths[1].cost == 440);
}

void HighLevelCBSTests::it_can_find_a_solution_in_a_graph_with_many_vertices(){
    // Arrange
    // We will construct a grid graph
    int gridWidth = 7;
    int gridHeight = 3;
    // Create vertices
    std::vector<std::shared_ptr<Vertex>> vertices{(long unsigned int)(gridWidth*gridHeight)};
    for (int i = 0; i < gridWidth; ++i){
        for (int j = 0; j < gridHeight; ++j){
            vertices[i + j * gridWidth] = std::make_shared<Vertex>(i + j*gridWidth);
        }
    }
    // Create edges
    for (int i = 0; i < gridWidth; i++){
        for (int j = 0; j < gridHeight; ++j){
            std::vector<std::shared_ptr<Edge>> edges;
            for (int k = i - 1; k <= i + 1; ++k){
                for (int l = j - 1; l <= j + 1; ++l){
                    // If we are within bounds
                    if (((k != i && l == j) || (l != j && k == i)) && !(k < 0 || k == gridWidth || l < 0 || l == gridHeight)){
                        edges.emplace_back(std::make_shared<Edge>(vertices[i + j * gridWidth], vertices[k + l * gridWidth], 100));
                    }
                }
            }
            vertices[i + j * gridWidth]->setEdges(edges);
        }
    }
    
    // for (int i = 0; i < gridWidth*gridHeight; i++){
    //     std::vector<std::shared_ptr<Edge>> edges; //Edges to various vertices
    //     if (i-1 >= 0 || i-1 != gridWidth) edges.emplace_back(std::make_shared<Edge>(vertices[i], vertices[i-1], 100)); //The vertex right befre
    //     if (i+1 < gridWidth*gridHeight) edges.emplace_back(std::make_shared<Edge>(vertices[i], vertices[i+1], 100)); //The vertex right after
    //     if (i+gridWidth < gridWidth*gridHeight) edges.emplace_back(std::make_shared<Edge>(vertices[i], vertices[i+gridWidth], 100)); //The vertex below
    //     if (i-gridWidth >= 0) edges.emplace_back(std::make_shared<Edge>(vertices[i], vertices[i-gridWidth], 100)); //The vertex above
    //     vertices[i]->setEdges(edges);
    //     std::cout << i << "-" << edges.size() << "\n";
    // }

    auto graph = std::make_shared<Graph>(vertices);

    //AgentInfo(id, action, dest)
    int agentCount = 1;
    std::vector<AgentInfo> agents{(long unsigned int)agentCount};
    for (int i = 0; i < agentCount; ++i){
        agents[i] = AgentInfo(i, Action(0, vertices[i], vertices[i], 0), vertices[gridWidth*gridHeight-1-i]);
    }
    
    Logger("HighLevel.txt");Logger("LowLevel.txt");

    // Act
    Solution solution = HighLevelCBS::get_instance().findSolution(graph, agents, LowLevelCBS::get_instance());

    // Assert
    assert((int)solution.paths.size() == agentCount);
    int i = 0;
    for (auto p : solution.paths){
        std::cout << "Path" << i << " cost: " << solution.paths[i].cost << "\n";
        i++;
    }
}