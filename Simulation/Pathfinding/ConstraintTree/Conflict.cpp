#include "Conflict.hpp"

std::vector<std::shared_ptr<Agent>> Conflict::getAgents(){
    //TODO implement properly :)
    std::vector<std::shared_ptr<Agent>> a{3};
    return a;
}
int Conflict::getTimeStart(){
    //TODO implement properly :)
    return 42;
}
int Conflict::getTimeEnd(){
    //TODO implement properly :)
    return 69;
}
Location Conflict::getLocation(){
    return this->location;
}