//
// Created by martin on 11/11/20.
//

#include "SingleThreadUppaalBot.hpp"

SingleThreadUppaalBot::SingleThreadUppaalBot():
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcPosition(NULL),
    m_cAlpha(7.5f),
    m_fDelta(0.1f),
    m_fWheelVelocity(100),
    m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                            ToRadians(m_cAlpha)) {}

void SingleThreadUppaalBot::Init(argos::TConfigurationNode& t_node) {
    /*
     * Get sensor/actuator handles
     *
     * The passed string (ex. "differential_steering") corresponds to the
     * XML tag of the device whose handle we want to have. For a list of
     * allowed values, type at the command prompt:
     *
     * $ argos3 -q actuators
     *
     * to have a list of all the possible actuators, or
     *
     * $ argos3 -q sensors
     *
     * to have a list of all the possible sensors.
     *
     * NOTE: ARGoS creates and initializes actuators and sensors
     * internally, on the basis of the lists provided the configuration
     * file at the <controllers><CFootBotDiffusion><actuators> and
     * <controllers><CFootBotDiffusion><sensors> sections. If you forgot to
     * list a device in the XML and then you request it here, an error
     * occurs.
     */
    m_pcWheels    = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <argos::CCI_FootBotProximitySensor      >("footbot_proximity"    );
    m_pcPosition  = GetSensor  <argos::CCI_PositioningSensor>("positioning");
    /*
     * Parse the configuration file
     *
     * The user defines this part. Here, the algorithm accepts three
     * parameters and it's nice to put them in the config file so we don't
     * have to recompile if we want to try other settings.
     */
    argos::GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
    m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
    argos::GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
    argos::GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

void SingleThreadUppaalBot::ControlStep(){
    constructInitialUppaalModel();
    return;
}

void SingleThreadUppaalBot::constructInitialUppaalModel(){
    std::ifstream partial_blueprint{std::string{std::filesystem::current_path()} + "/station_planning_blueprint.xml"};
    std::ofstream full_model{std::string{std::filesystem::current_path()} + "/initial_model.xml"};

    Robot self = sMap.getRobotByName(m_strId);
    std::vector<int> job = sMap.getNextJob();

    // This is the Uppaal model for the initial strategy.
    std::string line;
    int numOfStations = sMap.stationIDs.size() + sMap.endStationIDs.size() + 1;
    while(std::getline(partial_blueprint, line)){

        auto pos = line.find("#MAX_STATIONS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#MAX_STATIONS#"}.size(),
                std::to_string(numOfStations));
        }

        pos = line.find("#CUR_STATION#");
        if(pos != std::string::npos){
            //@todo: Make proper functions to encapsulate the number written!
            // The id matches the last index of the expanded DistMatrix.
            line.replace(pos, std::string{"#CUR_STATION#"}.size(),
                std::to_string(numOfStations - 1));
        }

        pos = line.find("#OTHER_ROBOTS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#OTHER_ROBOTS#"}.size(),
                         std::to_string(numOfOtherActiveRobots(sMap.Robots, self)));
        }


        pos = line.find("#CUR_ORDER#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#CUR_ORDER#"}.size(),
                         format_order(numOfStations, job));
        }

        std::string matrix = get_expanded_distance_matrix(sMap, self.getInitialLoc());

        pos = line.find("#DISTANCE_MATRIX#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#DISTANCE_MATRIX#"}.size(), matrix);
        }

        pos = line.find("#END_STATIONS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#END_STATIONS#"}.size(),
                format_endstations(numOfStations, sMap.endStationIDs));
        }

        full_model << line << std::endl;

    }
    exit(0);
}

REGISTER_CONTROLLER(SingleThreadUppaalBot, "SingleThreadUppaalBot_controller")