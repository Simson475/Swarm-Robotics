#include "AgentInfo.hpp"

int AgentInfo::getId(){ return id; }
void AgentInfo::setId(size_t id) { this->id = id; }
std::shared_ptr<Location> AgentInfo::getLocation()
{
    ExperimentData::get_instance().getAgents()[id]->getLocation();
    return location;
}
std::shared_ptr<Vertex> AgentInfo::getGoal(){ return goal; }