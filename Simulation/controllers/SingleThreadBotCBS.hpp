#ifndef SWARMSIMULATOR_SINGLETHREADBOTCBS_HPP
#define SWARMSIMULATOR_SINGLETHREADBOTCBS_HPP


#include "models/map/map_structure.hpp"
#include "models/jobs/JobGenerator.hpp"
#include "models/jobs/JobBlueprint.hpp"

#include "RobotInterface/RobotInterface.hpp"

/* Definition of the CCI_Controller class. */
#include "argos3/core/control_interface/ci_controller.h"
/* Definition of the differential steering actuator */
#include "argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h"
/* Definition of the foot-bot proximity sensor */
#include "argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h"
#include "argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h"
#include "argos3/core/simulator/loop_functions.h"
//  ???, this was added in order to get  GetID() robot
#include "argos3/plugins/robots/foot-bot/simulator/footbot_entity.h"

#include <vector>
#include <string>
#include <set>
#include <memory>

#include <functional>

class SingleThreadBotCBS : public virtual RobotInterface {

public:
    /* Class constructors. */
    using RobotInterface::RobotInterface;
    void Init(argos::TConfigurationNode &t_node) override;
    std::vector<Point>findOptimalPath();
    std::vector<int> receivedWaypointPlan;

protected:

    std::vector<int> constructStationPlan() override;
    std::vector<int> constructWaypointPlan() override;

private:
    void sortJob(const std::vector<std::vector<float>> &shortestDistances, std::vector<int>& job);
};


#endif //SWARMSIMULATOR_SINGLETHREADBOTCBS_HPP
