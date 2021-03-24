

#ifndef SWARMSIMULATOR_DEBUGBOT_HPP
#define SWARMSIMULATOR_DEBUGBOT_HPP

#include "RobotInterface/RobotInterface.hpp"
#include <queue>

class DebugBot : public RobotInterface {

public:
    /* Class constructors. */
    using RobotInterface::RobotInterface;

protected:

    std::vector<int> constructStationPlan() override;
    std::vector<int> constructWaypointPlan() override;

    void specialInit() override;

private:
    std::queue<std::vector<int>> stationPlans{};
    std::queue<std::vector<int>> waypointPlans{};


    void getPlans();
    bool isStationPlan(std::string);
    std::vector<int> extractPlanFromLine(std::string);

};


#endif //SWARMSIMULATOR_DEBUGBOT_HPP
