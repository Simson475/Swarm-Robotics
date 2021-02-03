#ifndef SWARMSIMULATOR_SINGLETHREADUPPAALBOT_HPP
#define SWARMSIMULATOR_SINGLETHREADUPPAALBOT_HPP

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

class SingleThreadUppaalBot : public RobotInterface {

public:
    /* Class constructors. */
    using RobotInterface::RobotInterface;

protected:

    std::vector<int> constructStationPlan() override;
    std::vector<int> constructWaypointPlan() override;

private:

    //**************** ControlStep functionality
    void constructStationUppaalModel(); // Must become an override
    void constructWaypointUppaalModel(); // Must become an override
    std::string runStationModel(); // Private
    std::string runWaypointModel(); // Private


    std::vector<int> getStationPlan(std::string modelOutput); // Private
    std::vector<int> getWaypointPlan(std::string modelOutput); // Private
};


#endif //SWARMSIMULATOR_SINGLETHREADUPPAALBOT_HPP
