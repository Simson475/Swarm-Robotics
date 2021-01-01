//
// Created by napalys on 12/16/20.
//

#include "SingleThreadBotGreedy.hpp"
#include "parsing/uppaal_model_parsing.hpp"
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

SingleThreadBotGreedy::SingleThreadBotGreedy() :
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

void SingleThreadBotGreedy::log_helper(const std::string &message, bool newLine, bool printName) {
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

void
SingleThreadBotGreedy::experiment_helper(const std::string &type, double time, int pointsToVisit, int pointsInPlan) {
    std::ofstream dataFile;
    dataFile.open(std::string{std::filesystem::current_path()} + "/data.csv", std::ofstream::app);

    dataFile << m_strId << ", " << type << ", " << std::to_string(time) << ", " << pointsToVisit << ", " << pointsInPlan
             << std::endl;
}

void SingleThreadBotGreedy::experiment_job_data(const std::string &type, int id, int logicalTime) {
    std::ofstream dataFile;
    dataFile.open(std::string{std::filesystem::current_path()} + "/data.csv", std::ofstream::app);

    dataFile << m_strId << ", " << type << ", " << std::to_string(logicalTime) << ", " << id << "," << std::endl;
}

void SingleThreadBotGreedy::Init(argos::TConfigurationNode &t_node) {
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

void SingleThreadBotGreedy::ControlStep() {
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
                log_helper("Constructs Station model");
                //constructStationUppaalModel();
                log_helper("Constructed Station model");

                auto t_start = std::chrono::high_resolution_clock::now();
                std::vector<int> stationPlan = getStationPlan("");
                auto t_end = std::chrono::high_resolution_clock::now();
                auto time_elapsed_s = std::chrono::duration<double, std::milli>(t_end - t_start).count() / 1000;
                experiment_helper("StationPlan", time_elapsed_s, currentJob->getRemainingStations().size(),
                                  stationPlan.size());
                log_helper("Station plan has size " + std::to_string(stationPlan.size()));

                setStationPlan(stationPlan);
                log_helper("Next station is now: " + std::to_string(getNextStation()));
            } else if (stationPlan.empty() && lastLocation != initLocation && returningToInit) {
                setStationPlan(std::vector<int>{initLocation});
            }


            if (hasJob() && waypointPlan.empty()) //@todo: Have proper boolean function
            {
                log_helper("Constructs Waypoint model");
                //constructWaypointUppaalModel();
                log_helper("Constructed Waypoint model");

                auto t_start = std::chrono::high_resolution_clock::now();
                std::vector<int> waypointPlan = getWaypointPlan("");
                auto t_end = std::chrono::high_resolution_clock::now();
                auto time_elapsed_s = std::chrono::duration<double, std::milli>(t_end - t_start).count() / 1000;
                experiment_helper("WaypointPlan", time_elapsed_s, 1, waypointPlan.size());

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
void SingleThreadBotGreedy::obtainOtherBots(Map_Structure &sMap) {
    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto &botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        auto &controller = dynamic_cast<SingleThreadBotGreedy &>(pcBot->GetControllableEntity().GetController());

        if (GetId() != controller.GetId())
            otherBots.push_back(controller);
    }
}

void SingleThreadBotGreedy::resetWaypointPlan() {
    waypointPlan.clear();
}

void SingleThreadBotGreedy::resetStationPlan() {
    log_helper("Clearing station plan");
    stationPlan.clear();
}

int SingleThreadBotGreedy::getNextStation() {
    return stationPlan.front();
}

int SingleThreadBotGreedy::getNextWaypoint() {
    return waypointPlan.front();
}

void SingleThreadBotGreedy::setJobGenerator(std::shared_ptr<JobGenerator> jobGenerator) {
    this->jobGenerator = std::move(jobGenerator);
}

bool SingleThreadBotGreedy::jobCompleted() {
    if (currentJob == nullptr)
        return false;

    return currentJob->isCompleted();
}

void SingleThreadBotGreedy::clearJob() {
    currentJob = nullptr;
}

bool SingleThreadBotGreedy::isAtStation() {
    const argos::CCI_PositioningSensor::SReading &tPosReads = m_pcPosition->GetReading();
    Point nextPoint = sMap.getPointByID(nextLocation);

    if (Distance(tPosReads.Position, nextPoint) <= 0.15) {
        log_helper("Has arrived at station " + std::to_string(nextLocation));
        return true;
    }
    return false;
}

bool SingleThreadBotGreedy::isStationNextInPlan(int stationId) {
    return stationId == stationPlan.front();
}

void SingleThreadBotGreedy::movementLogic() {
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

void SingleThreadBotGreedy::movementHelper(double per, double dotProd, double velocity) {
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

void SingleThreadBotGreedy::sortJob(const std::vector<std::vector<float>> &shortestDistances, std::vector<int> &job) {
    for (size_t i = 0; i < job.size() - 1; i++) {
        int k = i;
        float min = std::numeric_limits<float>::infinity();
        for (size_t j = i; j < job.size() - 1; j++) {
            float temp;
            if (i == 0) {
                temp = shortestDistances[nextLocation][job[j]];
            } else temp = shortestDistances[job[i - 1]][job[j]];

            if (temp < min) {
                min = temp;
                k = j;
            }
        }
        std::swap(job[i], job[k]);
    }
}

std::vector<int> SingleThreadBotGreedy::getStationPlan(std::string modelOutput) {
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

std::vector<int> SingleThreadBotGreedy::getWaypointPlan(std::string modelOutput) {
    std::vector<int> tempPlan{};

    auto plan = sMap.findPath(lastLocation, stationPlan.front());

    for (auto &p : plan) {
        tempPlan.emplace_back(p.getId());
    }

    return tempPlan;
}

void SingleThreadBotGreedy::setStationPlan(std::vector<int> stationPlan) {
    this->stationPlan = std::move(stationPlan);
}

void SingleThreadBotGreedy::setWaypointPlan(std::vector<int> waypointPlan) {
    this->waypointPlan = std::move(waypointPlan);
}

bool SingleThreadBotGreedy::hasJob() {
    return currentJob != nullptr;
}

unsigned int SingleThreadBotGreedy::sizeOfStationPlan() {
    return (unsigned) stationPlan.size();
}

int SingleThreadBotGreedy::getLastLocation() const {
    return lastLocation;
}

std::vector<int> SingleThreadBotGreedy::getStationPlan() {
    return stationPlan;
}

std::vector<int> SingleThreadBotGreedy::getWaypointPlan() {
    return waypointPlan;
}

std::set<int> SingleThreadBotGreedy::getOrder() {
    return currentJob->getRemainingStations();
}

void SingleThreadBotGreedy::setJob() {
    currentJob = jobGenerator->getNextJob();

    log_helper("Job is now: ", false);
    for (int j : currentJob->getRemainingStations()) {
        log_helper(std::to_string(j) + " ", false, false);
    }
    log_helper("", true, false);
}

void SingleThreadBotGreedy::setFinalJob() {
    currentJob = jobGenerator->generateGetHomeJob(initLocation);
    returningToInit = true;

    log_helper("Final job set");

    log_helper("Job is now: ", false);
    for (int j : currentJob->getRemainingStations()) {
        log_helper(std::to_string(j) + " ", false, false);
    }
    log_helper("", true, false);
}

void SingleThreadBotGreedy::setNextLocation(int locationID) {
    nextLocation = locationID;
}

void SingleThreadBotGreedy::setInitLocation(int locationID) {
    lastLocation = locationID;
    initLocation = locationID;
}

bool SingleThreadBotGreedy::isDoneWorking() const {
    return clock >= clockLimit;
}

void SingleThreadBotGreedy::startWorking(int clockLimit) {
    clock = 0;
    this->clockLimit = clockLimit;
    currentState = state::working;
}

void SingleThreadBotGreedy::setWorkingClockAsComplete() {
    currentState = state::moving;
}

void SingleThreadBotGreedy::advanceClock() {
    clock++;

    if (clock > clockLimit)
        throw std::logic_error("Working clock exceeds the limit of work to do.");
}

bool SingleThreadBotGreedy::isActive() {
    return hasJob() && !returningToInit;
}

bool SingleThreadBotGreedy::isWorking() {
    return currentState == state::working;
}

int SingleThreadBotGreedy::getClockCount() const {
    return clock;
}

REGISTER_CONTROLLER(SingleThreadBotGreedy, "SingleThreadUppaalBot_controller")