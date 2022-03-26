#include "Action.hpp"

void Action::operator=(const Action &a){
    timestamp = a.timestamp;
    startVertex = a.startVertex;
    endVertex = a.endVertex;
    duration = a.duration;
}