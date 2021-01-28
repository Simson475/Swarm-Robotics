

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

RobotInterface::RobotInterface() :
    m_pcWheels(nullptr),
    m_pcProximity(nullptr),
    m_pcPosition(nullptr),
    m_cAlpha(7.5f),
    m_fDelta(0.1f),
    m_fWheelVelocity(100),
    m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                            ToRadians(m_cAlpha)) {}


void print_string(const std::string &text, const std::string &fileName = "/debug.txt") {
    std::ofstream debug{std::string{std::filesystem::current_path()} + fileName};

    debug << text;

    debug.close();
}

void RobotInterface::log_helper(const std::string &message, bool newLine, bool printName) {
    std::ofstream logFile;
    logFile.open(std::string{std::filesystem::current_path()} + "/log.txt", std::ofstream::app);
    std::string name = printName ? m_strId + ": " : "";

    argos::LOG << name << message;
    logFile << name << message;

    if (newLine) {
        argos::LOG << std::endl;
        logFile << std::endl;
    }
}

void RobotInterface::experiment_helper(const std::string &type, double time, int pointsToVisit, int pointsInPlan) {
    std::ofstream dataFile;
    dataFile.open(std::string{std::filesystem::current_path()} + "/data.csv", std::ofstream::app);

    dataFile << m_strId << ", " << type << ", " << std::to_string(time) << ", " << pointsToVisit << ", " << pointsInPlan
             << std::endl;
}

void RobotInterface::experiment_job_data(const std::string &type, int id, int logicalTime) {
    std::ofstream dataFile;
    dataFile.open(std::string{std::filesystem::current_path()} + "/data.csv", std::ofstream::app);

    dataFile << m_strId << ", " << type << ", " << std::to_string(logicalTime) << ", " << id << "," << std::endl;
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

    currentState = state::moving;
}

void RobotInterface::ControlStep() {
    if (currentState == state::moving) {
        if (!stationPlan.empty() && isAtStation()) { // If we have a plan and we are at a point

            log_helper("Pre-reset");
            lastLocation = nextLocation;
            resetWaypointPlan();
            if (currentJob->isStationInJob(
                lastLocation)) { // Then we have reached the station @todo: Proper function for checking
                log_helper("Arrived at a work station");

                startWorking(50);
                sMap.setPointAsOccupied(lastLocation);
            } else if (isStationNextInPlan(lastLocation)) {
                resetStationPlan();
            }
            log_helper("Post-reset");
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
                }
            }

            if (hasJob() && stationPlan.empty() && !returningToInit) //@todo: Have proper boolean function
            {
                std::vector<int> stationPlan = constructStationPlan();

                log_helper("Station plan has size " + std::to_string(stationPlan.size()));
                setStationPlan(stationPlan);
                log_helper("Next station is now: " + std::to_string(getNextStation()));

            } else if (stationPlan.empty() && lastLocation != initLocation && returningToInit) {
                setStationPlan(std::vector<int>{initLocation});
            }

            if (hasJob() && waypointPlan.empty()) //@todo: Have proper boolean function
            {
                std::vector<int> waypointPlan = constructWaypointPlan();

                log_helper("Waypoint plan has size " + std::to_string(waypointPlan.size()));
                setWaypointPlan(waypointPlan);
                setNextLocation(waypointPlan.front());
                log_helper("Going towards " + std::to_string(nextLocation));
            }
        }
    } else if (currentState == state::working) {
        if (isDoneWorking()) {
            setWorkingClockAsComplete();

            currentJob->visitedStation(lastLocation);
            resetStationPlan();

            log_helper("Job is reduced: ", false);
            for (int j : currentJob->getRemainingStations()) {
                log_helper(std::to_string(j) + " ", false, false);
            }
            log_helper("", true, false);
            sMap.setPointAsAvailable(lastLocation);
        } else {
            advanceClock();
        }
    }

    movementLogic();
}

//Sets the vector of references of all the other robots in the system.
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
    argos::CVector3 newOri = nextPoint - tPosReads.Position; // Direct Access to Map
    newOri.Normalize();

    double per = newOri.GetX() * Ori.GetY() - newOri.GetY() * Ori.GetX();
    double dotProd = newOri.GetX() * Ori.GetX() + newOri.GetY() * Ori.GetY();


    if (Distance(tPosReads.Position, nextPoint) <= 2 && !sMap.isPointAvailable(nextPoint.getId())) {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    } else if (Distance(tPosReads.Position, nextPoint) <= 0.30) { // acceptance radius between point and robot
        movementHelper(per, dotProd, Distance(tPosReads.Position, nextPoint) * 60);
    } else {
        movementHelper(per, dotProd, m_fWheelVelocity);
    }

}

void RobotInterface::movementHelper(double per, double dotProd, double velocity) {
    const argos::CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
    argos::CVector2 cAccumulator;
    for (size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += argos::CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();
    /* If the angle of the vector is small enough and the closest obstacle
     * is far enough, continue going straight, otherwise curve a little
    */
    argos::Real turnRate;
    if (per > 0.5 || per < -0.5) { turnRate = 10.0f; }
    else { turnRate = 3.0f; }// while the angle is big our turn rate is fast
    if (per < 0.1 && per > -0.1) { turnRate = 1.0f; } //if the angle is small then our turn rate is reduced to 1
    argos::CRadians cAngle = cAccumulator.Angle();
    if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
        cAccumulator.Length() < m_fDelta) {
        if (per > 0.1) {
            m_pcWheels->SetLinearVelocity(turnRate, -turnRate);
        } else if (per < -0.1) {
            m_pcWheels->SetLinearVelocity(-turnRate, turnRate);
        } else {
            if (dotProd > 0) {
                m_pcWheels->SetLinearVelocity(velocity, velocity);
            } else {
                m_pcWheels->SetLinearVelocity(velocity, 0.0f);
            }
        }
    } else {


        /* Turn, depending on the sign of the angle */
        if (cAngle.GetValue() > 0.0f) {
            m_pcWheels->SetLinearVelocity(10.0f, 0.0f); // right
        } else {
            m_pcWheels->SetLinearVelocity(0.0f, 10.0f); // left
        }
    }
}

void RobotInterface::sortJob(const std::vector<std::vector<float>> &shortestDistances, std::vector<int> &job) {
    for (size_t i = 0; i < job.size() - 1; i++) {
        int k = i;
        float min = std::numeric_limits<float>::infinity();
        for (size_t j = i; j < job.size() - 1; j++) {
            float temp;
            if (i == 0) {
                temp = shortestDistances[lastLocation][job[j]];
            } else temp = shortestDistances[job[i - 1]][job[j]];

            if (temp < min) {
                min = temp;
                k = j;
            }
        }
        std::swap(job[i], job[k]);
    }
}

std::vector<int> RobotInterface::constructStationPlan() {
    std::vector<int> tempPlan{};
    if (!currentJob->getRemainingStations().empty()) {
        for (auto &job : currentJob->getRemainingStations())
            tempPlan.push_back(job);
        sortJob(sMap.getShortestDistanceMatrix(), tempPlan);
    } else {
        for (auto &job : currentJob->getEndStations())
            tempPlan.push_back(job);
    }
    return tempPlan;
}

std::vector<int> RobotInterface::constructWaypointPlan() {
    std::vector<int> tempPlan{};

    auto plan = sMap.findPath(lastLocation, stationPlan.front());

    for (auto &p : plan) {
        tempPlan.emplace_back(p.getId());
    }

    return tempPlan;
}

void RobotInterface::setStationPlan(std::vector<int> stationPlan) {
    this->stationPlan = std::move(stationPlan);
}

void RobotInterface::setWaypointPlan(std::vector<int> waypointPlan) {
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