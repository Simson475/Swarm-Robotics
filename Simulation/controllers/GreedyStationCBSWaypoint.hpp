

#ifndef SWARMSIMULATOR_GREEDYSTATIONCBSWAYPOINT_HPP
#define SWARMSIMULATOR_GREEDYSTATIONCBSWAYPOINT_HPP


#include "SingleThreadBotCBS.hpp"
#include "SingleThreadBotGreedy.hpp"

class GreedyStationCBSWaypoint : public SingleThreadBotCBS, SingleThreadBotGreedy {

public:
    /* Class constructors. */
    using SingleThreadBotCBS::SingleThreadBotCBS;

protected:

    std::vector<int> constructStationPlan() override;
    std::vector<int> constructWaypointPlan() override;

};


#endif
