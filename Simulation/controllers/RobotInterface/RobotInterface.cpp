

#include "RobotInterface.hpp"
#include "argos_wrapper/argos_wrapper.hpp"

#include <exception>
#include <cstdio>
#include <regex>
#include <fstream>
#include <set>
#include <iostream>
#include <filesystem>
#include <ctime>
#include <chrono>
#include <experimental/iterator>
#include <limits>

/** Class constructor */
RobotInterface::RobotInterface() :
    m_pcWheels(nullptr),
    m_pcProximity(nullptr),
    m_pcPosition(nullptr),
    m_cAlpha(10.0f),
    m_fDelta(0.02f),
    m_fWheelVelocity(100),
    m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                            ToRadians(m_cAlpha)) {}

/** Prints the string to the given filename relative to the current path of file system */
void RobotInterface::print_string(const std::string &text, const std::string &fileName) {
    std::ofstream debug{std::string{std::filesystem::current_path()} + fileName};

    debug << text;

    debug.close();
}

/** Opens the log.txt and prints the message to the log file and argos::LOG. 
  * If printName is true it prefixes with the id: 
  * If the newLine is set it ends the message with a newline.
 */
void RobotInterface::log_helper(const std::string &message, bool newLine, bool printName){
    std::ofstream logFile;
    logFile.open(std::string{std::filesystem::current_path()} + "/log.txt", std::ofstream::app);
    std::string name = printName ? m_strId + ": " : "";

    argos::LOG << name << message;
    logFile << name << message;

    if(newLine) {
        argos::LOG << std::endl;
        logFile << std::endl;
    }
}

/**
 * Appends 'ID, type, time, getLastLocation(), pointsToVisit, pointsInPlan' to the data.csv
*/
void RobotInterface::experiment_helper(const std::string &type, double time, int pointsToVisit, int pointsInPlan){
    std::ofstream dataFile;
    dataFile.open(std::string{std::filesystem::current_path()} + "/data.csv", std::ofstream::app);

    dataFile << m_strId << ", " << type << ", " << std::to_string(time) << ", " << getLastLocation() << ", " << pointsToVisit << ", " << pointsInPlan << std::endl;
}

/**
 * Appends 'ID, type, logicalTime, getLastLocation(), id' to the data.csv
*/
void RobotInterface::experiment_job_data(const std::string &type, int id, int logicalTime){
    std::ofstream dataFile;
    dataFile.open(std::string{std::filesystem::current_path()} + "/data.csv", std::ofstream::app);

    dataFile << m_strId << ", " << type << ", " << std::to_string(logicalTime) << ", " << getLastLocation() << ", " << id << "," << std::endl;
}

/**
 * Appends 'ID, type, , getLastLocation(), value_1, value_2' to the data.csv
*/
void RobotInterface::store_data(const std::string &type, const std::string& value_1, const std::string& value_2){
    std::ofstream dataFile;
    dataFile.open(std::string{std::filesystem::current_path()} + "/data.csv", std::ofstream::app);

    dataFile << m_strId << ", " << type << ", " << "" << ", " << getLastLocation() << ", " << value_1 << "," <<  value_2 <<std::endl;
}

void RobotInterface::Init(argos::TConfigurationNode &t_node) {
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
    m_pcWheels = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcPosition = GetSensor<argos::CCI_PositioningSensor>("positioning");
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


    argos::TConfigurationNode& params = argos::GetNode(t_node, "state");
    argos::GetNodeAttribute(params, "working_time", working_time);

    currentState = state::moving;

    specialInit();
}

/**
 * Controls how it does the jobs, chooses stations and moves.
*/
void RobotInterface::ControlStep() {
    time++;
    if (currentState == state::waiting) {
        wait();
    }
    if (currentState == state::working) {
        if (isDoneWorking()) {
            setWorkingClockAsComplete();

            currentJob->visitedStation(lastLocation);
            resetStationPlan();

            if (!currentJob->getRemainingStations().empty()) {
                log_helper("Job is reduced: ", false);
                for (int j : currentJob->getRemainingStations()) {
                    log_helper(std::to_string(j) + " ", false, false);
                }
                log_helper("", true, false);
            } else {
                log_helper("Job completed");
            }
            sMap.setPointAsAvailable(lastLocation);
        } else {
            advanceClock();
        }
    }

    if (currentState == state::moving) {
        if (!stationPlan.empty() && sMap.isPointAvailable(nextLocation) && isAtStation()) { // If we have a plan and we are at a point

            lastLocation = nextLocation;
            reachedPointEvent(lastLocation);

            if (currentJob->isStationInJob(lastLocation) && isStationNextInPlan(lastLocation)) { // Then we have reached the station @todo: Proper function for checking
                log_helper("Arrived at a work station");

                startWorking(working_time);
                sMap.setPointAsOccupied(lastLocation);
            } else if (isStationNextInPlan(lastLocation)) {
                resetStationPlan();
            }
        }

        if (currentState == state::moving) {
            if (!hasJob() || jobCompleted()) {
                if (jobCompleted()) {
                    currentJob->markAsCompleted();
                    experiment_job_data("EndedJob", currentJob->getID(),
                                        argos::CSimulator::GetInstance().GetSpace().GetSimulationClock());
                    clearJob();
                }
                if (jobGenerator->anyJobsLeft()) {
                    log_helper("Sets job");
                    setJob();
                    experiment_job_data("StartedJob", currentJob->getID(),
                                        argos::CSimulator::GetInstance().GetSpace().GetSimulationClock());
                } else if (lastLocation != initLocation) {
                    log_helper("Sets final job");
                    setFinalJob();
                }else{
                    currentState=state::finished;
                }
            }

            if (hasJob() && stationPlan.empty() && !returningToInit) //@todo: Have proper boolean function
            {
                /// Abstract function
                std::vector<int> stationPlan = constructStationPlan();

                log_helper("Station plan has size " + std::to_string(stationPlan.size()));
                storePlan(stationPlan, "Station");
                setStationPlan(stationPlan);
                log_helper("Station plan: ", false);
                for (int j : stationPlan) {
                    log_helper(std::to_string(j) + " ", false, false);
                }
                log_helper("", true, false);
                log_helper("Next station is now: " + std::to_string(getNextStation()));

            } else if (stationPlan.empty() && lastLocation != initLocation && returningToInit) {
                setStationPlan(std::vector<int>{initLocation});
            }

            if (hasJob() && waypointPlan.empty()) //@todo: Have proper boolean function
            {
                /// Abstract function
                std::vector<int> waypointPlan = constructWaypointPlan();
                
                if ( ! waypointPlan.empty()) {
                    log_helper("Waypoint plan has size " + std::to_string(waypointPlan.size()));
                    storePlan(waypointPlan, "Waypoint");
                    setWaypointPlan(waypointPlan);
                    setNextLocation(waypointPlan.front());
                    log_helper("Waypoint plan: ", false);
                    for (int j : waypointPlan) {
                        log_helper(std::to_string(j) + " ", false, false);
                    }
                    log_helper("", true, false);
                    log_helper("Going towards " + std::to_string(nextLocation));
                }
            }
        }
    }
    movementLogic();
}

/**
 * Prints the plans to the '_plans.csv' file as 'type; p1, p2, ..., pn'
*/
void RobotInterface::storePlan(std::vector<int> plan, std::string type) {
    std::ofstream planFile;
    planFile.open(std::string{std::filesystem::current_path()} + "/" + GetId() +"_plans.csv", std::ofstream::app);

    std::stringstream formatted_elements{};

    std::copy(plan.begin(),
              plan.end(),
              std::experimental::make_ostream_joiner(formatted_elements, ", "));
    formatted_elements << std::endl;

    planFile << type << "; ";
    planFile << formatted_elements.str();
}

/**
 * Sets the vector of references of all the other robots in the system.
 * The other bots are appended to the otherBots vector.
*/
void RobotInterface::obtainOtherBots(Map_Structure &sMap) {
    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto &botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        auto &controller = dynamic_cast<RobotInterface &>(pcBot->GetControllableEntity().GetController());

        if (GetId() != controller.GetId())
            otherBots.push_back(controller);
    }
}

void RobotInterface::resetWaypointPlan() {
    waypointPlan.clear();
}

void RobotInterface::resetStationPlan() {
    log_helper("Clearing station plan");
    stationPlan.clear();
}

int RobotInterface::getNextStation() {
    return stationPlan.front();
}

int RobotInterface::getNextWaypoint() {
    return waypointPlan.front();
}

void RobotInterface::setJobGenerator(std::shared_ptr<JobGenerator> jobGenerator) {
    this->jobGenerator = std::move(jobGenerator);
}

bool RobotInterface::jobCompleted() {
    if (currentJob == nullptr)
        return false;

    return currentJob->isCompleted();
}

void RobotInterface::clearJob() {
    if (returningToInit) {
        log_helper("Robot's state is now 'done'");
        currentState = state::done;
    }
    currentJob = nullptr;
}

bool RobotInterface::isAtStation() {
    const argos::CCI_PositioningSensor::SReading &tPosReads = m_pcPosition->GetReading();
    Point nextPoint = sMap.getPointByID(nextLocation);

    if (Distance(tPosReads.Position, nextPoint) <= 0.15) {
        log_helper("Has arrived at station " + std::to_string(nextLocation));
        return true;
    }
    return false;
}

bool RobotInterface::isStationNextInPlan(int stationId) {
    return stationId == stationPlan.front();
}

void RobotInterface::movementLogic() {
    const argos::CCI_PositioningSensor::SReading &tPosReads = m_pcPosition->GetReading();

    Point nextPoint = sMap.getPointByID(nextLocation);
    argos::CRadians a, c, b;
    tPosReads.Orientation.ToEulerAngles(a, b, c);

    double oy = sin(a.GetValue());
    double ox = cos(a.GetValue());
    argos::CVector3 Ori(ox, oy, 0);
    argos::CVector3 newOri = getDestDirection();

    double crossProd = newOri.GetX() * Ori.GetY() - newOri.GetY() * Ori.GetX(); // @todo: Use newOri.CrossProduct(..) when merge is accepted.
    double dotProd = Ori.DotProduct(newOri);

    if (Distance(tPosReads.Position, nextPoint) <= 1.3 && !sMap.isPointAvailable(nextPoint.getId())) {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    } else if (Distance(tPosReads.Position, nextPoint) <= 0.30) { // acceptance radius between point and robot
        movementHelper(crossProd, dotProd, Distance(tPosReads.Position, nextPoint) * 60);
    } else {
        movementHelper(crossProd, dotProd, m_fWheelVelocity);
    }
}

void RobotInterface::movementHelper(double crossProd, double dotProd, double velocity) {
    /* If the angle of the vector is small enough and the closest obstacle
     * is far enough, continue going straight, otherwise curve a little
    */
    argos::Real turnRate;

    // Sets the turn speed based on the difference on the angle between robot's front and direction of dest
    if (crossProd > 0.5 || crossProd < -0.5)
        turnRate = 10.0f;
    else
        turnRate = 3.0f;

    if(otherBotIsTooClose() && !isPathBlocked()){
        argos::CVector2 orientation = getOrientation2D();
        argos::CVector2 others_orientation = getCloseRobotsDirection();

        if(radianBetweenDirections(orientation, others_orientation).GetValue() > 0){
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0);
        } else {
            m_pcWheels->SetLinearVelocity(0, m_fWheelVelocity*0.8);
        }
    }
    else if(isPathBlocked() && isFacingDest()){
        if (isBlockageOnTheSide()) {
            // Robot continues as long blockage is on its side.
            m_pcWheels->SetLinearVelocity(velocity, velocity);
        } else {
            // The robot change direction depending on the sign of the blockage concerning dest orientation
            argos::CVector2 blockOrientation = getBlockOrientation();
            if(radianBetweenDirections(getDestDirection2D(), blockOrientation).GetValue() > 0){
                m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0);
            } else {
                m_pcWheels->SetLinearVelocity(0, m_fWheelVelocity*0.8);
            }
        }
    } else {
        //There is no block and the robot corrects its orientation according to the destination
        if (crossProd > 0.1) {
            m_pcWheels->SetLinearVelocity(turnRate, -turnRate);
        } else if (crossProd < -0.1) {
            m_pcWheels->SetLinearVelocity(-turnRate, turnRate);
        } else {
            if (dotProd > 0) {
                m_pcWheels->SetLinearVelocity(velocity, velocity);
            } else {
                m_pcWheels->SetLinearVelocity(velocity, 0.0f);
            }
        }
    }
}


argos::CVector2 RobotInterface::getProximityVector(){
    // Get Robot's normalized direction vector
    argos::CVector2 orientation = getOrientation2D();
    argos::CVector2 destDirection = getDestDirection2D();

    //Sets up for getting readings
    const argos::CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
    argos::CVector2 cAccumulator{};
    size_t numOfReadings = 0;


    for(auto reading : tProxReads){
        argos::CVector2 sensorOrientation = orientation;
        sensorOrientation.Rotate(reading.Angle);


        if(radianBetweenDirections(destDirection, sensorOrientation).GetAbsoluteValue() < ARGOS_PI/2) {
            cAccumulator += argos::CVector2(reading.Value, reading.Angle);
            numOfReadings++;
        }
    }

    assert(numOfReadings > 0); // Otherwise, no readings are used.
    cAccumulator /= numOfReadings;

    return cAccumulator;
}

argos::CVector2 RobotInterface::getPosition() {
    const argos::CCI_PositioningSensor::SReading &tPosReads = m_pcPosition->GetReading();

    return argos::CVector2{tPosReads.Position.GetX(), tPosReads.Position.GetY()};
}

bool RobotInterface::otherBotIsTooClose(){
    argos::CVector2 pos = getPosition();

    for (auto& other_bot : otherBots) {
        if(other_bot.get().isActive()) {
            argos::CVector2 others_pos = other_bot.get().getPosition();

            if ((pos - others_pos).Length() < 0.085036758f * 2 &&
                radianBetweenDirections(others_pos - pos, getOrientation2D()).GetAbsoluteValue() <
                ARGOS_PI / 2) { //Value is the radius of robots and is in "footbot_entity.cpp"
                //log_helper(other_bot.get().GetId() + " is " + std::to_string((pos - others_pos).Length()) + " away");
                return true;
            }
        }
    }

    return false;
}

argos::CVector2 RobotInterface::getCloseRobotsDirection(){
    argos::CVector2 pos = getPosition();


    double tmp_dist = std::numeric_limits<double>::infinity();
    argos::CVector2 tmp_pos{};


    for (auto& other_bot : otherBots) {

        argos::CVector2 others_pos = other_bot.get().getPosition();

        if((pos - others_pos).Length() < tmp_dist) { //Value is the radius of robots and is in "footbot_entity.cpp"
            tmp_dist = (pos - others_pos).Length();
            tmp_pos = others_pos;
        }
    }

    return argos::CVector2{tmp_pos - pos};
}


bool RobotInterface::isPathBlocked(){
    return getProximityVector().Length() >= m_fDelta;
}

bool RobotInterface::isFacingDest(){
    argos::CVector2 orientation = getOrientation2D();
    argos::CVector2 destDirection = getDestDirection2D();

    return radianBetweenDirections(orientation, destDirection).GetAbsoluteValue() < ARGOS_PI / 2;
}


// Gets a vector that returns the robot's orientation.
argos::CVector2 RobotInterface::getOrientation2D() {
    const argos::CCI_PositioningSensor::SReading &tPosReads = m_pcPosition->GetReading();

    argos::CVector2 test;

    argos::CRadians a, c, b;
    tPosReads.Orientation.ToEulerAngles(a, b, c);

    double oy = sin(a.GetValue());
    double ox = cos(a.GetValue());
    argos::CVector2 orientation = argos::CVector2(ox, oy);
    orientation.Normalize();
    return orientation;
}

bool RobotInterface::isBlockageOnTheSide() {
    // Get Robot's normalized direction vector
    argos::CVector2 destDirection = getDestDirection2D();

    argos::CVector2 blockDirection = getProximityVector();

    if (blockDirection.Angle().GetAbsoluteValue() > ARGOS_PI/2 - 0.2){
        argos::CVector2 orientation = getOrientation2D();
        orientation.Rotate(blockDirection.Angle());
        return radianBetweenDirections(orientation, destDirection).GetAbsoluteValue() < ARGOS_PI/2;
    }

    return false;
}

argos::CVector2 RobotInterface::getBlockOrientation(){
    argos::CVector2 orientation = getOrientation2D();

    return orientation.Rotate(radianOfBlock());
}

argos::CRadians RobotInterface::radianOfBlock() {
    return getProximityVector().Angle();
}

argos::CVector3 RobotInterface::getDestDirection() {
    const argos::CCI_PositioningSensor::SReading &tPosReads = m_pcPosition->GetReading();

    Point nextPoint = sMap.getPointByID(nextLocation);

    argos::CVector3 destDirection = nextPoint - tPosReads.Position; // Direct Access to Map
    destDirection.Normalize();
    
    return destDirection;
}

argos::CVector2 RobotInterface::getDestDirection2D() {
    const argos::CCI_PositioningSensor::SReading &tPosReads = m_pcPosition->GetReading();

    Point nextPoint = sMap.getPointByID(nextLocation);

    argos::CVector2 destDirection = argos::CVector2(nextPoint.getX() - tPosReads.Position.GetX(),
                                                    nextPoint.getY() - tPosReads.Position.GetY());
    destDirection.Normalize();

    return destDirection;
}

argos::CRadians RobotInterface::radianBetweenDirections(argos::CVector2 fst, argos::CVector2 snd){
    return argos::ATan2(fst.GetX()*snd.GetY()-fst.GetY()*snd.GetX(),
        fst.GetX()*snd.GetX()+fst.GetY()*snd.GetY());
}

void RobotInterface::setStationPlan(std::vector<int> stationPlan) {
    this->stationPlan = std::move(stationPlan);
}

void RobotInterface::setWaypointPlan(std::vector<int> waypointPlan) {
    lastModification = getLogicalTime();
    this->waypointPlan = std::move(waypointPlan);
}

bool RobotInterface::hasJob() {
    return currentJob != nullptr;
}

unsigned int RobotInterface::sizeOfStationPlan() {
    return (unsigned) stationPlan.size();
}

int RobotInterface::getLastLocation() const {
    return lastLocation;
}

std::vector<int> RobotInterface::getStationPlan() {
    return stationPlan;
}

std::vector<int> RobotInterface::getWaypointPlan() {
    return waypointPlan;
}

std::set<int> RobotInterface::getOrder() {
    return currentJob->getRemainingStations();
}

void RobotInterface::setJob() {
    currentJob = jobGenerator->getNextJob();

    log_helper("Job is now: ", false);
    for (int j : currentJob->getRemainingStations()) {
        log_helper(std::to_string(j) + " ", false, false);
    }
    log_helper("", true, false);
}

void RobotInterface::setFinalJob() {
    currentJob = jobGenerator->generateGetHomeJob(initLocation);
    returningToInit = true;

    log_helper("Final job set");

    log_helper("Job is now: ", false);
    for (int j : currentJob->getRemainingStations()) {
        log_helper(std::to_string(j) + " ", false, false);
    }
    log_helper("", true, false);
}


void RobotInterface::setNextLocation(int locationID) {
    nextLocation = locationID;
}

void RobotInterface::setInitLocation(int locationID) {
    lastLocation = locationID;
    initLocation = locationID;
}

bool RobotInterface::isDoneWorking() const {
    return clock >= clockLimit;
}

void RobotInterface::startWorking(int clockLimit) {
    clock = 0;
    this->clockLimit = clockLimit;
    currentState = state::working;
}

void RobotInterface::setWorkingClockAsComplete() {
    currentState = state::moving;
}

void RobotInterface::advanceClock() {
    clock++;

    if (clock > clockLimit)
        throw std::logic_error("Working clock exceeds the limit of work to do.");
}

bool RobotInterface::isActive() {
    return hasJob() && !returningToInit;
}

bool RobotInterface::isWorking() {
    return currentState == state::working;
}

int RobotInterface::getClockCount() const {
    return clock;
}

// If the robot is active and there has been no activity for 10 logical minutes, the robot is in a deadlock.
bool RobotInterface::isInLivelock() {
    if(currentState == state::done)
        return false;

    return lastModification + 30 * 60 * 10 < getLogicalTime();
}

void RobotInterface::reachedPointEvent(int id){
    resetWaypointPlan();
}

void RobotInterface::wait(){
    //Do nothing
}

bool RobotInterface::isFinished(){
    return this->currentState == state::finished;
}