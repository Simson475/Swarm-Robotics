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
        Action(500, v1, v2, 1000),
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
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    auto v3 = std::make_shared<Vertex>(3);
    a1 = Action(0, v1, v2, 100);
    a2 = Action(0, v3, v2, 100);

    // Act & Assert
    // Meets at the same vertex within delta time
    assert(ct.isVertexConflict(a1, a2));

    // a1 moves to the vertex a2 is waiting at
    a2 = Action(0, v2, v2, 100);
    assert(ct.isVertexConflict(a1, a2));

    // a2 moves to the vertex a1 is waiting at
    a1 = Action(0, v2, v2, 100);
    a2 = Action(0, v1, v2, 100);
    assert(ct.isVertexConflict(a1, a2));

    // Going to same vertex, but at different times (still overlapping)
    a1 = Action(0, v1, v2, 500);
    a2 = Action(0, v3, v2, 100);
    assert( ! ct.isVertexConflict(a1, a2));
}

void ConstraintTreeTests::it_is_edge_conflict(){
    // Arrange
    ConstraintTree ct;
    Action a1, a2;
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v2, 100),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v1, 100),
    };
    v1->setEdges(v1edges);
    v2->setEdges(v2edges);
    a1 = Action(0, v1, v2, 100);
    a2 = Action(50, v1, v2, 100);

    // Act & Assert
    assert(ct.isEdgeConflict(a1, a2));
}

void ConstraintTreeTests::it_is_swap_conflict(){
    // Arrange
    ConstraintTree ct;
    Action a1, a2;
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v2, 100),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v1, 100),
    };
    v1->setEdges(v1edges);
    v2->setEdges(v2edges);
    a1 = Action(0, v1, v2, 100);
    a2 = Action(0, v2, v1, 100);

    // Act & Assert
    assert(ct.isSwapConflict(a1, a2));
    a2 = Action(50, v2, v1, 100);
    assert(ct.isSwapConflict(a1, a2));
}

void ConstraintTreeTests::it_is_follow_conflict(){
    // Arrange
    ConstraintTree ct;
    Action a1, a2;
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    auto v3 = std::make_shared<Vertex>(3);

    // Act & Assert
    // Meets at the same vertex within delta time
    a1 = Action(0, v1, v2, 100);
    a2 = Action(100, v2, v3, 100);
    assert(ct.isFollowConflict(a1, a2));

    // a1 moves to the vertex a2 is waiting at
    a2 = Action(a1.timestamp + a1.duration, v2, v2, 100);
    assert(ct.isFollowConflict(a1, a2));

    // a1 moves to the vertex a2 waited at > delta ago
    a2 = Action(a1.timestamp + a1.duration + DELTA + 1, v2, v2, 100);
    assert( ! ct.isFollowConflict(a1, a2));
}

void ConstraintTreeTests::it_can_get_vertex_conflict(){
    // Arrange
    ConstraintTree ct;
    Action a1, a2;
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    auto v3 = std::make_shared<Vertex>(3);
    a1 = Action(0, v1, v2, 100);
    a2 = Action(10, v3, v2, 100);
    std::vector<int> agents = {1, 2};

    // Act
    Conflict c1 = ct.getVertexConflict(agents, a1, a2);

    // Assert
    assert(c1.getAgentIds()[0] == agents[0]);
    assert(c1.getAgentIds()[1] == agents[1]);
    assert(c1.getLocation().toString() == "v2");
    assert(c1.getTimeStart() == a1.timestamp + a1.duration);
    assert(c1.getTimeEnd() == (a1.timestamp + a1.duration + DELTA));
}

void ConstraintTreeTests::it_can_get_follow_conflict(){
    // Arrange
    ConstraintTree ct;
    Action a1, a2;
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    auto v3 = std::make_shared<Vertex>(3);
    a1 = Action(0, v1, v2, 100);
    a2 = Action(110, v2, v3, 100);
    std::vector<int> agents = {1, 2};

    // Act
    Conflict c1 = ct.getFollowConflict(agents, a1, a2);

    // Assert
    assert(c1.getAgentIds()[0] == agents[0]);
    assert(c1.getAgentIds()[1] == agents[1]);
    assert(c1.getLocation().toString() == "v2");
    assert(c1.getTimeStart() == a2.timestamp);
    assert(c1.getTimeEnd() == (a1.timestamp + a1.duration + DELTA));
}

void ConstraintTreeTests::it_can_get_edge_conflict(){
    // Arrange
    ConstraintTree ct;
    Action a1, a2;
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v2, 100),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v1, 100),
    };
    v1->setEdges(v1edges);
    v2->setEdges(v2edges);
    a1 = Action(0, v1, v2, 100);
    a2 = Action(50, v1, v2, 100);
    std::vector<int> agents = {1, 2};

    // Act
    Conflict c1 = ct.getEdgeConflict(agents, a1, a2);

    // Assert
    assert(c1.getAgentIds()[0] == agents[0]);
    assert(c1.getAgentIds()[1] == agents[1]);
    assert(c1.getLocation().toString() == "e[1,2]");
    assert(c1.getTimeStart() == a2.timestamp);
    assert(c1.getTimeEnd() == (a1.timestamp + a1.duration));
}

void ConstraintTreeTests::it_can_get_swap_conflict(){
    // Arrange
    ConstraintTree ct;
    Action a1, a2;
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    std::vector<std::shared_ptr<Edge>> v1edges = {
        std::make_shared<Edge>(v1, v2, 100),
    };
    std::vector<std::shared_ptr<Edge>> v2edges = {
        std::make_shared<Edge>(v2, v1, 100),
    };
    v1->setEdges(v1edges);
    v2->setEdges(v2edges);
    a1 = Action(0, v1, v2, 100);
    a2 = Action(50, v2, v1, 100);
    std::vector<int> agents = {1, 2};

    // Act
    Conflict c1 = ct.getSwapConflict(agents[0], a1, a2);
    Conflict c2 = ct.getSwapConflict(agents[1], a2, a1);

    // Assert
    assert(c1.getAgentIds()[0] == agents[0]);
    assert(c2.getAgentIds()[0] == agents[1]);
    assert(c1.getLocation().toString() == "e[1,2]");
    assert(c2.getLocation().toString() == "e[2,1]");
    assert(c1.getTimeStart() == a2.timestamp);
    assert(c1.getTimeEnd() == (a1.timestamp + a1.duration));
    assert(c1.getTimeStart() == c2.getTimeStart());
    assert(c1.getTimeEnd() == c2.getTimeEnd());
}