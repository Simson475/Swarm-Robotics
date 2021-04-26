

#ifndef SWARMSIMULATOR_UPPAALSTATIONGREEDYWAYPOINT_HPP
#define SWARMSIMULATOR_UPPAALSTATIONGREEDYWAYPOINT_HPP

#include "SingleThreadUppaalBot.hpp"
#include "SingleThreadBotGreedy.hpp"

class UppaalStationGreedyWaypoint : public SingleThreadUppaalBot, SingleThreadBotGreedy {

public:
    /* Class constructors. */
    using SingleThreadUppaalBot::SingleThreadUppaalBot;

protected:

    std::vector<int> constructStationPlan() override;
    std::vector<int> constructWaypointPlan() override;

};


#endif //SWARMSIMULATOR_UPPAALSTATIONGREEDYWAYPOINT_HPP
