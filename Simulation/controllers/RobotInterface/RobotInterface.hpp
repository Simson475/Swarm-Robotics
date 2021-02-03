#ifndef SWARMSIMULATOR_ROBOTINTERFACE_HPP
#define SWARMSIMULATOR_ROBOTINTERFACE_HPP

#include "models/map/map_structure.hpp"
#include "models/jobs/JobGenerator.hpp"
#include "models/jobs/JobBlueprint.hpp"

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

class RobotInterface : public argos::CCI_Controller {

public:
    /* Class constructor. */
    RobotInterface();

    /* Class destructor. */
    ~RobotInterface() override = default;

    /*
     * This function initializes the controller.
     * The 't_node' variable points to the <parameters> section in the XML
     * file in the <controllers><CFootBotDiffusion_controller> section.
     */
    void Init(argos::TConfigurationNode &t_node) override;

    /*
     * This function is called once every time step.
     * The length of the time step is set in the XML file.
     */
    void ControlStep() override;

    // Sets the job generator that is shared between the main loop function and all controllers.
    void setJobGenerator(std::shared_ptr<JobGenerator> jobGenerator);

    // Set the robots initial location
    void setInitLocation(int);

    // Obtain references for the other robots for information extraction when creating Uppaal models.
    void obtainOtherBots(Map_Structure&);


    // Functions the other controllers need
    bool hasJob();
    unsigned int sizeOfStationPlan();
    int getLastLocation() const;
    std::vector<int> getStationPlan();
    std::set<int> getOrder();
    std::vector<int> getWaypointPlan();
    int getNextStation();
    int getNextWaypoint();
    bool isActive();
    bool isWorking();
    int getClockCount() const;

    // Is the robot in a live deadlock
    bool isInLiveDeadlock();


protected:
    //Internal state for when to move and when to move
    enum class state {working, moving, done};

    state currentState;

    /* Pointer to the differential steering actuator */
    argos::CCI_DifferentialSteeringActuator *m_pcWheels;
    /* Pointer to the foot-bot proximity sensor */
    argos::CCI_FootBotProximitySensor *m_pcProximity;

    argos::CCI_PositioningSensor *m_pcPosition;

    /*
     * The following variables are used as parameters for the
     * algorithm. You can set their value in the <parameters> section
     * of the XML configuration file, under the
     * <controllers><CFootBotDiffusion_controller> section.
     */

    /* Maximum tolerance for the angle between
     * the robot heading direction and
     * the closest obstacle detected. */
    argos::CDegrees m_cAlpha;
    /* Maximum tolerance for the proximity reading between
     * the robot and the closest obstacle.
     * The proximity reading is 0 when nothing is detected
     * and grows exponentially to 1 when the obstacle is
     * and grows exponentially to 1 when the obstacle is
     * touching the robot.
     */
    argos::Real m_fDelta;
    /* Wheel speed. */
    argos::Real m_fWheelVelocity;
    /* Angle tolerance range to go straight.
     * It is set to [-alpha,alpha]. */
    argos::CRange<argos::CRadians> m_cGoStraightAngleRange;

    Map_Structure &sMap = Map_Structure::get_instance();
    std::shared_ptr<JobGenerator> jobGenerator;
    std::unique_ptr<JobBlueprint> currentJob; //Does not compile with unique_ptr

    std::vector<int> stationPlan{};
    std::vector<int> waypointPlan{};
    std::vector<std::reference_wrapper<RobotInterface>> otherBots{};
    int nextLocation;
    int lastLocation;
    int initLocation;
    bool returningToInit = false;

    // Used as a clock counter for when working at stations
    int clock;
    int clockLimit;

    // Clock for checking if robot is in a live deadlock
    int lastModification = 0;

    //**************** ControlStep functionality
    virtual std::vector<int> constructStationPlan() = 0;
    virtual std::vector<int> constructWaypointPlan() = 0;


    void setStationPlan(std::vector<int>);
    void setWaypointPlan(std::vector<int> waypointPlan);

    void setJob();
    void setFinalJob();
    void clearJob();
    bool jobCompleted();

    void setNextLocation(int);
    void movementLogic();
    void movementHelper(double per, double dotProd, double velocity);
    bool isAtStation();
    bool isStationNextInPlan(int);
    void resetWaypointPlan();
    void resetStationPlan();

    // Helper functions for debug and data printing.
    void print_string(const std::string &text, const std::string &fileName = "/debug.txt");
    void log_helper(const std::string& message, bool newLine=true, bool printName=true);
    void experiment_helper(const std::string& type, double time, int pointsToVisit, int pointsInPlan);
    void experiment_job_data(const std::string& type, int id, int logicalTime);

    //Clock/working functionality
    bool isDoneWorking() const;
    void startWorking(int clockLimit);
    void setWorkingClockAsComplete();
    void advanceClock();

};


#endif //SWARMSIMULATOR_ROBOTINTERFACE_HPP
