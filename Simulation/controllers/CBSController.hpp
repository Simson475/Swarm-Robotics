#ifndef SWARMSIMULATOR_CBSController_HPP
#define SWARMSIMULATOR_CBSController_HPP


#include "SingleThreadBotGreedy.hpp"

class CBSController : public SingleThreadBotGreedy {

public:
    /* Class constructors. */
    using SingleThreadBotGreedy::SingleThreadBotGreedy;

protected:

    std::vector<int> constructStationPlan() override;
    std::vector<int> constructWaypointPlan() override;

};


#endif //SWARMSIMULATOR_CBSController_HP
