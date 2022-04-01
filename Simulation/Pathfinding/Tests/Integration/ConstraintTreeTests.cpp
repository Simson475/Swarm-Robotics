#include "ConstraintTreeTests.hpp"
#include <queue>

void ConstraintTreeTests::it_can_find_swap_conflicts(){
    // Arrange
    ConstraintTree ct;
    Solution solution;
    Path p1, p2;
    /**
     * We will construct a L shaped graph
     * 1--2
    */
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v2, 1000),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v1, 1000),
    };
    v1->setEdges(v1edges);
    v2->setEdges(v2edges);
    p1.actions = {
        Action(0, v1, v2, 1000),
    };
    p2.actions = {
        Action(0, v2, v1, 1000),
    };
    solution.paths = { p1, p2 };
    ct.setSolution(solution);

    // Act
    auto conflicts = ct.findConflicts();

    // Assert
    assert(conflicts.size() == 2);
    assert(conflicts[0].getAgentIds().size() == 1);
    assert(conflicts[1].getAgentIds().size() == 1); 
    assert(conflicts[0].getLocation().toString() == "e[1,2]");
    assert(conflicts[0].getTimeStart() == 0);
    assert(conflicts[0].getTimeEnd() == 1000); 
    assert(conflicts[1].getLocation().toString() == "e[2,1]");
    assert(conflicts[1].getTimeStart() == 0);
    assert(conflicts[1].getTimeEnd() == 1000);
}

void ConstraintTreeTests::it_can_find_edge_conflicts(){
    // Arrange
    ConstraintTree ct;
    Solution solution;
    Path p1, p2;
    /**
     * We will construct a L shaped graph
     * 1--2
    */
    auto v1 = std::make_shared<Vertex>(0);
    auto v2 = std::make_shared<Vertex>(1);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v2, 1000),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v1, 1000),
    };
    p1.actions = {
        Action(100, v1, v2, 1100),
    };
    p2.actions = {
        Action(0, v1, v2, 1000),
    };
    solution.paths = { p1, p2 };
    ct.setSolution(solution);

    // Act
    auto conflicts = ct.findConflicts();

    // Assert
    assert(conflicts.size() == 1);
    //TODO more asserts
}

void ConstraintTreeTests::it_can_find_vertex_conflicts(){
    // Arrange
    ConstraintTree ct;
    Solution solution;
    Path p1, p2;
    /**
     * We will construct the following graph
     * 1--2
    */
    auto v1 = std::make_shared<Vertex>(0);
    auto v2 = std::make_shared<Vertex>(1);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v2, 1),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v1, 1),
    };
    p1.actions = {
        Action(0, v1, v2, 1),
    };
    p2.actions = {
        Action(1, v2, v2, 2),
    };
    solution.paths = { p1, p2 };
    ct.setSolution(solution);

    // Act
    auto conflicts = ct.findConflicts();

    // Assert
    assert(conflicts.size() == 1);
    //TODO more asserts
}

void ConstraintTreeTests::it_gets_sorted_in_priority_queue(){
    // Arrange
    /**
     * We will construct the following graph
     * 1--2----3-------4
    */
    auto v1 = std::make_shared<Vertex>(0);
    auto v2 = std::make_shared<Vertex>(1);
    auto v3 = std::make_shared<Vertex>(1);
    auto v4 = std::make_shared<Vertex>(1);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v2, 1),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v3, 4),
        std::make_shared<Edge>(v2, v1, 1),
    };
    std::vector<std::shared_ptr<Edge>> v3edges = {
        std::make_shared<Edge>(v3, v2, 4),
        std::make_shared<Edge>(v3, v4, 7),
    };
    std::vector<std::shared_ptr<Edge>> v4edges = {
        std::make_shared<Edge>(v4, v3, 7),
    };
    Solution s1, s2, s3;
    Path p1, p2, p3;
    p1.actions = {
        Action(0, v1, v2, 1),
    };
    p1.cost = 1;
    p2.actions = {
        Action(0, v2, v3, 4),
    };
    p2.cost = 4;
    p3.actions = {
        Action(0, v3, v4, 7),
    };
    p3.cost = 7;
    s1.paths = { p1 };
    s2.paths = { p2 };
    s3.paths = { p3 };
    auto ct3 = std::make_shared<ConstraintTree>();
    auto ct1 = std::make_shared<ConstraintTree>();
    auto ct2 = std::make_shared<ConstraintTree>();
    ct1->setSolution(s1);
    ct2->setSolution(s2);
    ct3->setSolution(s3);

    // Act
    std::priority_queue<std::shared_ptr<ConstraintTree>, std::vector<std::shared_ptr<ConstraintTree>>, ConstraintTree> pq;
    pq.push(ct2);
    pq.push(ct3);
    pq.push(ct1);

    // Assert
    assert(pq.top() == ct1);
    pq.pop();
    assert(pq.top() == ct2);
    pq.pop();
    assert(pq.top() == ct3);
    pq.pop();
}

void ConstraintTreeTests::it_is_vertex_conflict(){
    // Arrange
    ConstraintTree ct;
    Action a1, a2;
    /**
     * We will construct the following graph
     * 1--2--3
    */
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    auto v3 = std::make_shared<Vertex>(3);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v2, 100),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v1, 100),
        std::make_shared<Edge>(v2, v3, 100),
    };
    std::vector<std::shared_ptr<Edge>> v3edges = {
        std::make_shared<Edge>(v3, v2, 100),
    };
    v1->setEdges(v1edges);
    v2->setEdges(v2edges);
    v3->setEdges(v3edges);
    a1 = Action(0, v1, v2, 100);
    a2 = Action(0, v3, v2, 100);

    // Act & Assert
    std::cout << "Assert 1\n";
    assert(ct.isVertexConflict(a1, a2)); // Meet at the same vertex within delta time
    a2 = Action(21, v3, v2, 100);
    std::cout << "Assert 2\n";
    assert( ! ct.isVertexConflict(a1, a2)); // Go to the same vertex, but dont meet within delta time
    a2 = Action(121, v2, v3, 100);
    std::cout << "Assert 3\n";
    assert( ! ct.isVertexConflict(a1, a2)); // Follow, but with > delta time spacing
}

void ConstraintTreeTests::it_is_edge_conflict(){
    // Arrange
    ConstraintTree ct;
    Action a1, a2;
    /**
     * We will construct the following graph
     * 1--2--3
    */
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    auto v3 = std::make_shared<Vertex>(3);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v2, 100),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v1, 100),
        std::make_shared<Edge>(v2, v3, 100),
    };
    std::vector<std::shared_ptr<Edge>> v3edges = {
        std::make_shared<Edge>(v3, v2, 100),
    };
    v1->setEdges(v1edges);
    v2->setEdges(v2edges);
    v3->setEdges(v3edges);
    a1 = Action(0, v1, v2, 100);
    a2 = Action(50, v1, v2, 100);

    // Act & Assert
    assert(ct.isEdgeConflict(a1, a2));
}

void ConstraintTreeTests::it_is_swap_conflict(){
    // Arrange
    ConstraintTree ct;
    Action a1, a2;
    /**
     * We will construct the following graph
     * 1--2--3
    */
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    auto v3 = std::make_shared<Vertex>(3);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v2, 100),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v1, 100),
        std::make_shared<Edge>(v2, v3, 100),
    };
    std::vector<std::shared_ptr<Edge>> v3edges = {
        std::make_shared<Edge>(v3, v2, 100),
    };
    v1->setEdges(v1edges);
    v2->setEdges(v2edges);
    v3->setEdges(v3edges);
    a1 = Action(0, v1, v2, 100);
    a2 = Action(0, v2, v1, 100);

    // Act & Assert
    assert(ct.isSwapConflict(a1, a2));
}