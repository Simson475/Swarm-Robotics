

#ifndef SWARMSIMULATOR_GREEDYSTATIONCBSWAYPOINT_HPP
#define SWARMSIMULATOR_GREEDYSTATIONCBSWAYPOINT_HPP


#include "SingleThreadBotCBS.hpp"
#include "SingleThreadBotGreedy.hpp"
/* Definition of the CCI_Controller class. */
//#include "argos3/core/control_interface/ci_controller.h"

class GreedyStationCBSWaypoint : public SingleThreadBotGreedy, SingleThreadBotCBS {

public:
    /* Class constructors. */
    using SingleThreadBotCBS::SingleThreadBotCBS;

protected:

    std::vector<int> constructStationPlan() override;
    std::vector<int> constructWaypointPlan() override;

};


#endif
