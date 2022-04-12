#include "ActionPathAuxTests.hpp"

void ActionPathAuxTests::it_can_compare_in_priority_queue(){
    // Arrange
    auto v = std::make_shared<Vertex>(0);
    Action a = Action(0, v, v, 0);
    std::priority_queue<ActionPathAux, std::vector<ActionPathAux>, std::greater<ActionPathAux>> priorityQueue{};

    // Act
    priorityQueue.push(ActionPathAux(a, 30, nullptr));
    priorityQueue.push(ActionPathAux(a, 10, nullptr));
    priorityQueue.push(ActionPathAux(a, 20, nullptr));

    // Assert 
    auto aux = priorityQueue.top();priorityQueue.pop();
    assert(aux.heuristic == 10);
    aux = priorityQueue.top();priorityQueue.pop();
    assert(aux.heuristic == 20);
    aux = priorityQueue.top();priorityQueue.pop();
    assert(aux.heuristic == 30);
}

void ActionPathAuxTests::it_can_get_path(){
    // Arrange
    auto v0 = std::make_shared<Vertex>(0);
    auto v1 = std::make_shared<Vertex>(1);
    auto v2 = std::make_shared<Vertex>(2);
    auto v3 = std::make_shared<Vertex>(3);
    Action a0 = Action(0, v0, v1, 1);
    Action a1 = Action(1, v1, v2, 2);
    Action a2 = Action(2, v2, v2, 3);
    Action a3 = Action(3, v2, v3, 4);
    auto ap0 = std::make_shared<ActionPathAux>(a0, 1, nullptr);
    auto ap1 = std::make_shared<ActionPathAux>(a1, 1, ap0);
    auto ap2 = std::make_shared<ActionPathAux>(a2, 1, ap1);
    auto ap3 = std::make_shared<ActionPathAux>(a3, 1, ap2);

    // Act
    Path p = ap3->getPath();

    // Assert 
    assert(p.actions[0] == a0);
    assert(p.actions[1] == a1);
    assert(p.actions[2] == a2);
    assert(p.actions[3] == a3);
    assert(p.cost == TIME_AT_GOAL + 10);
}