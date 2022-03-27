#include "ActionPathAuxTests.hpp"

void ActionPathAuxTests::it_can_compare_in_priority_queue(){
    // Arrange
    auto v = std::make_shared<Vertex>(0);
    Action a = Action(0, v, v, 0);
    std::priority_queue<ActionPathAux> priorityQueue{};

    // Act
    priorityQueue.push(ActionPathAux(a, 30, nullptr));
    priorityQueue.push(ActionPathAux(a, 10, nullptr));
    priorityQueue.push(ActionPathAux(a, 20, nullptr));

    // Assert 
    auto aux = priorityQueue.top();priorityQueue.pop();
    assert(aux.priority == 10);
    aux = priorityQueue.top();priorityQueue.pop();
    assert(aux.priority == 20);
    aux = priorityQueue.top();priorityQueue.pop();
    assert(aux.priority == 30);
}