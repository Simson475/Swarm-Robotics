

#ifndef SWARMSIMULATOR_GREEDYSTATIONUPPAALWAYPOINT_HPP
#define SWARMSIMULATOR_GREEDYSTATIONUPPAALWAYPOINT_HPP


#include "SingleThreadUppaalBot.hpp"
#include "SingleThreadBotGreedy.hpp"

class GreedyStationUppaalWaypoint : public SingleThreadUppaalBot, SingleThreadBotGreedy {

public:
    /* Class constructors. */
    using SingleThreadUppaalBot::SingleThreadUppaalBot;

protected:

    std::vector<int> constructStationPlan() override;
    std::vector<int> constructWaypointPlan() override;

};


#endif //SWARMSIMULATOR_GREEDYSTATIONUPPAALWAYPOINT_HPP
