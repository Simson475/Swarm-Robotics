//
// Created by martin on 11/11/20.
//

#include "SingleThreadUppaalBot.hpp"
#include "parsing/uppaal_model_parsing.hpp"
#include "argos_wrapper/argos_wrapper.hpp"

#include <exception>
#include <cstdio>
#include <regex>
#include <fstream>
#include <set>
#include <iostream>
#include <filesystem>

SingleThreadUppaalBot::SingleThreadUppaalBot():
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcPosition(NULL),
    m_cAlpha(7.5f),
    m_fDelta(0.1f),
    m_fWheelVelocity(100),
    m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                            ToRadians(m_cAlpha)) {}


void print_string(std::string text, std::string fileName="/debug.txt"){
    std::ofstream debug{std::string{std::filesystem::current_path()} + fileName};

    debug << text;

    debug.close();
}

void SingleThreadUppaalBot::log_helper(std::string message, bool newLine, bool printName){
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

    currentState = state::moving;
}

void SingleThreadUppaalBot::ControlStep(){
    if(currentState == state::moving) {
        if (!stationPlan.empty() && isAtStation()) { // If we have a plan and we are at a point

            log_helper("Pre-reset");
            lastLocation = nextLocation;
            resetWaypointPlan();
            if (currentJob->isStationInJob(lastLocation)) { // Then we have reached the station @todo: Proper function for checking
                log_helper("Arrived at a work station");

                startWorking(50);
            } else if (isStationNextInPlan(lastLocation)) {
                resetStationPlan();
            }
            log_helper("Post-reset");
        }

        if(currentState == state::moving) {
            if (!hasJob() || jobCompleted()) {
                if (jobCompleted()) {
                    currentJob->markAsCompleted();
                    clearJob();
                }
                if (jobGenerator->anyJobsLeft()) {
                    log_helper("Sets job");
                    setJob();
                } else if (lastLocation != initLocation) {
                    log_helper("Sets final job");
                    setFinalJob();
                }
            }

            if (hasJob() && stationPlan.empty() && !returningToInit) //@todo: Have proper boolean function
            {
                log_helper("Constructs Station model");
                constructStationUppaalModel();
                log_helper("Constructed Station model");
                std::vector<int> stationPlan = getStationPlan(runStationModel());
                log_helper("Station plan has size " + std::to_string(stationPlan.size()));

                setStationPlan(stationPlan);
                log_helper("Next station is now: " + std::to_string(getNextStation()));
            } else if (stationPlan.empty() && lastLocation != initLocation && returningToInit) {
                setStationPlan(std::vector<int>{initLocation});
            }


            if (hasJob() && waypointPlan.empty()) //@todo: Have proper boolean function
            {
                log_helper("Constructs Waypoint model");
                constructWaypointUppaalModel();
                log_helper("Constructed Waypoint model");
                std::vector<int> waypointPlan = getWaypointPlan(runWaypointModel());
                log_helper("Waypoint plan has size " + std::to_string(waypointPlan.size()));
                setWaypointPlan(waypointPlan);
                setNextLocation(waypointPlan.front());
                log_helper("Going towards " + std::to_string(nextLocation));
            }
        }
    }
    else if(currentState == state::working){
        if(isDoneWorking()){
            setWorkingClockAsComplete();

            currentJob->visitedStation(lastLocation);
            resetStationPlan();

            log_helper("Job is reduced: ", false);
            for (int j : currentJob->getRemainingStations()) {
                log_helper(std::to_string(j) + " ", false, false);
            }
            log_helper("", true, false);
        }
        else {
            advanceClock();
        }
    }

    movementLogic();
}

//Sets the vector of references of all the other robots in the system.
void SingleThreadUppaalBot::obtainOtherBots(Map_Structure& sMap){
    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto& botPair : tBotMap) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(botPair.second);
        auto& controller = dynamic_cast<SingleThreadUppaalBot&>(pcBot->GetControllableEntity().GetController());

        if(GetId() != controller.GetId())
            otherBots.push_back(controller);
    }
}

void SingleThreadUppaalBot::resetWaypointPlan(){
    waypointPlan.clear();
}

void SingleThreadUppaalBot::resetStationPlan(){
    log_helper("Clearing station plan");
    stationPlan.clear();
}

int SingleThreadUppaalBot::getNextStation(){
    return stationPlan.front();
}

int SingleThreadUppaalBot::getNextWaypoint(){
    return waypointPlan.front();
}

void SingleThreadUppaalBot::setJobGenerator(std::shared_ptr<JobGenerator> jobGenerator){
    this->jobGenerator = std::move(jobGenerator);
}

bool SingleThreadUppaalBot::jobCompleted(){
    if(currentJob == nullptr)
        return false;

    return currentJob->isCompleted();
}

void SingleThreadUppaalBot::clearJob(){
    currentJob = nullptr;
}

bool SingleThreadUppaalBot::isAtStation(){
    const argos::CCI_PositioningSensor::SReading& tPosReads  = m_pcPosition->GetReading();
    Point nextPoint = sMap.getPointByID(nextLocation);

    if (Distance(tPosReads.Position,nextPoint) <= 0.15){
        log_helper("Has arrived at station " + std::to_string(nextLocation));
        return true;
    }
    return false;
}

bool SingleThreadUppaalBot::isStationNextInPlan(int stationId){
    return stationId == stationPlan.front();
}

void SingleThreadUppaalBot::movementLogic(){
    const argos::CCI_PositioningSensor::SReading& tPosReads  = m_pcPosition->GetReading();

    Point nextPoint = sMap.getPointByID(nextLocation);
    argos::CRadians a,c,b;
    tPosReads.Orientation.ToEulerAngles(a,b,c);

    double oy = sin(a.GetValue());
    double ox = cos(a.GetValue());
    argos::CVector3 Ori(ox,oy,0);
    argos::CVector3 newOri = nextPoint - Point(tPosReads.Position, pointType::tempCalculation, ""); // Direct Access to Map
    newOri.Normalize();

    double per = newOri.GetX()*Ori.GetY() - newOri.GetY()*Ori.GetX() ;
    double dotProd = newOri.GetX()*Ori.GetX() + newOri.GetY()*Ori.GetY();

    if(Distance(tPosReads.Position,nextPoint) <= 1.0 ){ // acceptance radius between point and robot
        movementHelper(per, dotProd, Distance(tPosReads.Position, nextPoint) * 60);
    }
    else {
        movementHelper(per, dotProd, m_fWheelVelocity);
    }

}

void SingleThreadUppaalBot::movementHelper(double per, double dotProd, double velocity){
    const argos::CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    argos::CVector2 cAccumulator;
    for(size_t i = 0; i < tProxReads.size(); ++i){
        cAccumulator += argos::CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();
    /* If the angle of the vector is small enough and the closest obstacle
     * is far enough, continue going straight, otherwise curve a little
    */
    argos::Real turnRate;
    if(per>0.5 || per<-0.5) {turnRate = 10.0f;} else {turnRate = 3.0f;}// while the angle is big our turn rate is fast
    if(per<0.1 && per>-0.1) {turnRate = 1.0f;} //if the angle is small then our turn rate is reduced to 1
    argos::CRadians cAngle = cAccumulator.Angle();
    if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
       cAccumulator.Length() < m_fDelta ){
        if( per > 0.1 ){
            m_pcWheels->SetLinearVelocity(turnRate, -turnRate );
        }
        else if ( per < -0.1 ){
            m_pcWheels->SetLinearVelocity(-turnRate, turnRate);
        }
        else{
            if (dotProd > 0){
                m_pcWheels->SetLinearVelocity(velocity, velocity);
            }
            else{
                m_pcWheels->SetLinearVelocity(velocity, 0.0f);
            }
        }
    }
    else{


        /* Turn, depending on the sign of the angle */
        if(cAngle.GetValue() > 0.0f) {
            m_pcWheels->SetLinearVelocity(10.0f, 0.0f); // right
        }
        else {
            m_pcWheels->SetLinearVelocity( 0.0f, 10.0f); // left
        }
    }
}


std::vector<int> SingleThreadUppaalBot::getStationPlan(std::string modelOutput) {
    std::smatch m;
    std::regex queryForm ("cur_station:\n");
    std::regex_search(modelOutput, m, queryForm);


    std::string queryResult = modelOutput.substr(m.position());
    std::ofstream debug2{std::string{std::filesystem::current_path()} + "/debug2.txt"};

    debug2 << modelOutput << "\n\n";
    debug2 << queryResult;

    std::set<int> stationsVisited{};
    std::vector<int> stationPlan{};


    std::regex queryPart(R"(([0-9]+)([.][0-9])?,([0-9]+))");
    std::regex_search(queryResult, m, queryPart);
    for (auto it = std::sregex_iterator(queryResult.begin(), queryResult.end(), queryPart);
        it != std::sregex_iterator(); it++){
        m = *it;
        debug2 << m[0] << ": ->  "  << m[3] << std::endl;

        //Needed for the initial creation of station plans.
        int tmpLastLocation;
        if(lastLocation >= sMap.getAmountOfStations())
            tmpLastLocation = sMap.stationIDs.size() + sMap.endStationIDs.size();
        else
            tmpLastLocation = lastLocation;

        if(m[1] != "0" && stationsVisited.find(std::stoi(m[3])) == stationsVisited.end()){
            if(stationPlan.empty() && std::stoi(m[3]) == tmpLastLocation) {
                continue;
            }
            stationPlan.push_back(std::stoi(m[3]));
            stationsVisited.insert(std::stoi(m[3]));
        }
    }

    debug2 << "Station plan:\n";
    for(int s : stationPlan)
        debug2 << s << std::endl;


    debug2.close();
    return stationPlan;
}

std::vector<int> SingleThreadUppaalBot::getWaypointPlan(std::string modelOutput) {
    std::smatch m;
    std::regex queryForm ("cur_station:\n");
    std::regex_search(modelOutput, m, queryForm);


    std::string queryResult = modelOutput.substr(m.position());
    std::ofstream debug4{std::string{std::filesystem::current_path()} + "/debug4.txt"};

    debug4 << modelOutput << "\n\n";
    debug4 << queryResult;

    std::set<int> stationsVisited{};
    std::vector<int> stationPlan{};


    std::regex queryPart(R"(([0-9]+)([.][0-9])?,([0-9]+))");
    std::regex_search(queryResult, m, queryPart);
    for (auto it = std::sregex_iterator(queryResult.begin(), queryResult.end(), queryPart);
         it != std::sregex_iterator(); it++){
        m = *it;
        debug4 << m[0] << ": ->  "  << m[3] << std::endl;

        int currentPosition = lastLocation; //Needs to be generalized to the current position in the given matrix.

        if(m[1] != "0" && stationsVisited.find(std::stoi(m[3])) == stationsVisited.end()){
            if(stationPlan.empty() && std::stoi(m[3]) == currentPosition) {
                continue;
            }
            stationPlan.push_back(std::stoi(m[3]));
            stationsVisited.insert(std::stoi(m[3]));
        }
    }

    debug4 << "Station plan:\n";
    for(int s : stationPlan)
        debug4 << s << std::endl;


    debug4.close();
    return stationPlan;
}

void SingleThreadUppaalBot::setStationPlan(std::vector<int> stationPlan){
    this->stationPlan = std::move(stationPlan);
}

void SingleThreadUppaalBot::setWaypointPlan(std::vector<int> waypointPlan){
    this->waypointPlan = std::move(waypointPlan);
}

std::string SingleThreadUppaalBot::runStationModel(){
    std::string verifyta{"~/Desktop/uppaalStratego/bin-Linux/verifyta.bin"};
//    std::string verifyta{"/home/martin/Desktop/uppaalStratego/bin-Linux/verifyta.bin"};
    //std::string verifyta{"~/phd/Uppaal/uppaal64-4.1.20-stratego-7/bin-Linux/verifyta"};
    std::string model{"./station_model.xml"};

    std::string terminalCommand = verifyta + " " + model;

    std::string result;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    stream = popen(terminalCommand.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != nullptr) result.append(buffer);
        pclose(stream);
    }

    print_string(result);
    return result;
}

std::string SingleThreadUppaalBot::runWaypointModel(){
    std::string verifyta{"~/Desktop/uppaalStratego/bin-Linux/verifyta.bin"};
    //std::string verifyta{"/home/martin/Desktop/uppaalStratego/bin-Linux/verifyta.bin"};
    std::string model{"./waypoint_model.xml"};

    std::string terminalCommand = verifyta + " " + model;

    std::string result;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    stream = popen(terminalCommand.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != nullptr) result.append(buffer);
        pclose(stream);
    }

    print_string(result);
    return result;
}

bool SingleThreadUppaalBot::hasJob() {
    return currentJob != nullptr;
}

unsigned int SingleThreadUppaalBot::sizeOfStationPlan(){
    return (unsigned)stationPlan.size();
}

int SingleThreadUppaalBot::getLastLocation(){
    return lastLocation;
}

std::vector<int> SingleThreadUppaalBot::getStationPlan(){
    return stationPlan;
}

std::vector<int> SingleThreadUppaalBot::getWaypointPlan(){
    return waypointPlan;
}

std::set<int> SingleThreadUppaalBot::getOrder(){
    return currentJob->getRemainingStations();
}

void SingleThreadUppaalBot::setJob() {
    currentJob = jobGenerator->getNextJob();

    log_helper("Job is now: ", false);
    for(int j : currentJob->getRemainingStations()){
        log_helper(std::to_string(j) + " ", false, false);
    }
    log_helper("", true, false);
}

void SingleThreadUppaalBot::setFinalJob() {
    currentJob = jobGenerator->generateGetHomeJob(initLocation);
    returningToInit = true;

    log_helper("Final job set");

    log_helper("Job is now: ", false);
    for(int j : currentJob->getRemainingStations()){
        log_helper(std::to_string(j) + " ", false, false);
    }
    log_helper("", true, false);
}

void SingleThreadUppaalBot::setNextLocation(int locationID){
    nextLocation = locationID;
}

void SingleThreadUppaalBot::setInitLocation(int locationID){
    lastLocation = locationID;
    initLocation = locationID;
}

bool SingleThreadUppaalBot::isDoneWorking(){
    return clock >= clockLimit;
}

void SingleThreadUppaalBot::startWorking(int clockLimit){
    clock = 0;
    this->clockLimit = clockLimit;
    currentState = state::working;
}

void SingleThreadUppaalBot::setWorkingClockAsComplete(){
    currentState = state::moving;
}

void SingleThreadUppaalBot::advanceClock(){
    clock++;

    if (clock > clockLimit)
        throw std::logic_error("Working clock exceeds the limit of work to do.");
}

void SingleThreadUppaalBot::constructStationUppaalModel(){
    std::ifstream partial_blueprint{std::string{std::filesystem::current_path()} + "/planning_blueprint.xml"};
    std::ofstream full_model{std::string{std::filesystem::current_path()} + "/station_model.xml"};

    // This is the Uppaal model for the initial strategy.
    std::string line;
    int numOfStations;
    std::string matrix;

    if(lastLocation >= sMap.getAmountOfStations()){
        matrix = get_expanded_distance_matrix(sMap, lastLocation);
        numOfStations = sMap.stationIDs.size() + sMap.endStationIDs.size() + 1;
    }
    else {
        matrix = formatMatrix(sMap.floydShortestOfStations());
        numOfStations = sMap.stationIDs.size() + sMap.endStationIDs.size();
    }

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

            if(lastLocation >= sMap.getAmountOfStations()){
                line.replace(pos, std::string{"#CUR_STATION#"}.size(),
                             std::to_string(sMap.getAmountOfStations()));
            }
            else
                line.replace(pos, std::string{"#CUR_STATION#"}.size(),
                    std::to_string(lastLocation));
        }

        pos = line.find("#OTHER_ROBOTS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#OTHER_ROBOTS#"}.size(),
                         std::to_string(numOfOtherActiveRobots(otherBots)));
        }


        pos = line.find("#CUR_ORDER#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#CUR_ORDER#"}.size(),
                         format_order(numOfStations, currentJob->getRemainingStations()));
        }

        pos = line.find("#DISTANCE_MATRIX#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#DISTANCE_MATRIX#"}.size(), matrix);
        }

        pos = line.find("#END_STATIONS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#END_STATIONS#"}.size(),
                format_endstations(numOfStations, currentJob->getEndStations()));
        }

        pos = line.find("#XML_COMMENT_START#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#XML_COMMENT_START#"}.size(), "<!--");
            else
                line.replace(pos, std::string{"#XML_COMMENT_START#"}.size(), "");
        }


        pos = line.find("#XML_COMMENT_END#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#XML_COMMENT_END#"}.size(), "-->");
            else
                line.replace(pos, std::string{"#XML_COMMENT_END#"}.size(), "");
        }

        pos = line.find("#CODE_COMMENT_START#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#CODE_COMMENT_START#"}.size(), "/*");
            else
                line.replace(pos, std::string{"#CODE_COMMENT_START#"}.size(), "");
        }

        pos = line.find("#CODE_COMMENT_END#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#CODE_COMMENT_END#"}.size(), "*/");
            else
                line.replace(pos, std::string{"#CODE_COMMENT_END#"}.size(), "");
        }

        pos = line.find("#REQUIRE_ENDSTATIONS_START#");
        if(pos != std::string::npos){
                line.replace(pos, std::string{"#REQUIRE_ENDSTATIONS_START#"}.size(), "");
        }

        pos = line.find("#REQUIRE_ENDSTATIONS_END#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#REQUIRE_ENDSTATIONS_END#"}.size(), "");
        }

        pos = line.find("#END_AT_ENDSTATION#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#END_AT_ENDSTATION#"}.size(), "end_stations[s]");
        }


        pos = line.find("#OTHER_IN_SYSTEM#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#OTHER_IN_SYSTEM#"}.size(), "");
            else {
                line.replace(pos, std::string{"#OTHER_IN_SYSTEM#"}.size(), "OtherRobot, ");
            }

        }

        pos = line.find("#QUERY#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#QUERY#"}.size(),
                         format_query(numOfStations));
        }

        pos = line.find("#QUERY_TIME#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#QUERY_TIME#"}.size(),
                         "2000");
        }

        //********************* Helper functions for when there are other active robots
        if(numOfOtherActiveRobots(otherBots) > 0) {
            pos = line.find("#OTHER_ORDER_LENGHT#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_ORDER_LENGHT#"}.size(),
                             formatStationOrderLenghts(otherBots));
            }

            pos = line.find("#OTHER_START_LOCS#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_START_LOCS#"}.size(),
                             formatOrthersStartLocs(otherBots));
            }

            pos = line.find("#OTHER_PLANS#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_PLANS#"}.size(),
                             formatOtherStationPlan(otherBots, numOfStations));
            }

            pos = line.find("#OTHER_ORDERS#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_ORDERS#"}.size(),
                             formatOtherOrders(otherBots, numOfStations));
            }

            pos = line.find("#OTHER_DISTANCES#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_DISTANCES#"}.size(),
                             formatOtherStationDistances(otherBots, sMap));
            }

        }
        full_model << line << std::endl;

    }

    full_model.close();
}

void SingleThreadUppaalBot::constructWaypointUppaalModel(){
    std::ifstream partial_blueprint{std::string{std::filesystem::current_path()} + "/planning_blueprint.xml"};
    std::ofstream waypoint_model{std::string{std::filesystem::current_path()} + "/waypoint_model.xml"};

    // This is the Uppaal model for the initial strategy.
    std::string line;

    while(std::getline(partial_blueprint, line)){

        auto pos = line.find("#MAX_STATIONS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#MAX_STATIONS#"}.size(),
                         std::to_string(sMap.points.size())); //@Todo: Have proper getter!
        }

        pos = line.find("#CUR_STATION#");
        if(pos != std::string::npos){
            //@todo: Make proper functions to encapsulate the number written!
            // The id matches the last index of the expanded DistMatrix.
            line.replace(pos, std::string{"#CUR_STATION#"}.size(),
                         std::to_string(lastLocation));
        }

        pos = line.find("#OTHER_ROBOTS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#OTHER_ROBOTS#"}.size(),
                         std::to_string(numOfOtherActiveRobots(otherBots)));
        }


        std::vector<std::vector<float>> matrix = getDistanceMatrix(sMap);

        pos = line.find("#CUR_ORDER#");
        if (pos != std::string::npos) {
            std::set<int> nextStation{};
            nextStation.insert(stationPlan.front());
            line.replace(pos, std::string{"#CUR_ORDER#"}.size(),
                         format_order(matrix.size(), nextStation));
        }

        pos = line.find("#DISTANCE_MATRIX#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#DISTANCE_MATRIX#"}.size(), formatMatrix(matrix));
        }

        pos = line.find("#END_STATIONS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#END_STATIONS#"}.size(),
                         format_endstations(matrix.size(), currentJob->getEndStations()));
        }

        pos = line.find("#XML_COMMENT_START#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#XML_COMMENT_START#"}.size(), "<!--");
            else
                line.replace(pos, std::string{"#XML_COMMENT_START#"}.size(), "");
        }

        pos = line.find("#XML_COMMENT_END#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#XML_COMMENT_END#"}.size(), "-->");
            else
                line.replace(pos, std::string{"#XML_COMMENT_END#"}.size(), "");
        }

        pos = line.find("#CODE_COMMENT_START#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#CODE_COMMENT_START#"}.size(), "/*");
            else
                line.replace(pos, std::string{"#CODE_COMMENT_START#"}.size(), "");
        }

        pos = line.find("#CODE_COMMENT_END#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#CODE_COMMENT_END#"}.size(), "*/");
            else
                line.replace(pos, std::string{"#CODE_COMMENT_END#"}.size(), "");
        }

        pos = line.find("#REQUIRE_ENDSTATIONS_START#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#REQUIRE_ENDSTATIONS_START#"}.size(), "/*");
        }

        pos = line.find("#REQUIRE_ENDSTATIONS_END#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#REQUIRE_ENDSTATIONS_END#"}.size(), "*/");
        }

        pos = line.find("#END_AT_ENDSTATION#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#END_AT_ENDSTATION#"}.size(), "true");
        }

        pos = line.find("#OTHER_IN_SYSTEM#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(otherBots) == 0)
                line.replace(pos, std::string{"#OTHER_IN_SYSTEM#"}.size(), "");
            else {
                line.replace(pos, std::string{"#OTHER_IN_SYSTEM#"}.size(), "OtherRobot, ");
            }
        }

        pos = line.find("#QUERY#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#QUERY#"}.size(),
                         format_query(sMap.points.size())); //@todo: Make proper getter!
        }

        pos = line.find("#QUERY_TIME#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#QUERY_TIME#"}.size(),
                         "500");
        }

        //********************* Helper functions for when there are other active robots
        if(numOfOtherActiveRobots(otherBots) > 0) {
            int numOfStations = sMap.points.size();
            pos = line.find("#OTHER_ORDER_LENGHT#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_ORDER_LENGHT#"}.size(),
                             formatWaypointOrderLenghts(otherBots));
            }

            pos = line.find("#OTHER_START_LOCS#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_START_LOCS#"}.size(),
                             formatOrtherWaypointStartLocs(otherBots));
            }

            pos = line.find("#OTHER_PLANS#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_PLANS#"}.size(),
                             formatOtherWaypointPlan(otherBots, numOfStations));
            }

            pos = line.find("#OTHER_ORDERS#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_ORDERS#"}.size(),
                             formatOtherWaypointOrders(otherBots, numOfStations));
            }

            pos = line.find("#OTHER_DISTANCES#");
            if (pos != std::string::npos) {
                line.replace(pos, std::string{"#OTHER_DISTANCES#"}.size(),
                             formatOtherStationDistances(otherBots, sMap));
            }
        }

        waypoint_model << line << std::endl;

    }

    waypoint_model.close();
}

REGISTER_CONTROLLER(SingleThreadUppaalBot, "SingleThreadUppaalBot_controller")