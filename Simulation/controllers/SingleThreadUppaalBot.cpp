//
// Created by martin on 11/11/20.
//

#include "SingleThreadUppaalBot.hpp"

#include <exception>
#include <cstdio>
#include <regex>
#include <fstream>
#include <set>
#include <iostream>

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

void test_function(){
    argos::CSpace::TMapPerType &tBotMap =
        argos::CLoopFunctions().GetSpace().GetEntitiesByType("foot-bot");
    for (auto it = tBotMap.begin(); it != tBotMap.end();
         ++it) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(it->second);
        //argos::CCI_Controller& controller = pcBot->GetControllableEntity().GetController();
        SingleThreadUppaalBot& testClass = dynamic_cast<SingleThreadUppaalBot&>(pcBot->GetControllableEntity().GetController());

        testClass.hurra = "Good job!";
    }

    for (auto it = tBotMap.begin(); it != tBotMap.end();
         ++it) {
        argos::CFootBotEntity *pcBot = argos::any_cast<argos::CFootBotEntity *>(it->second);
        //argos::CCI_Controller& controller = pcBot->GetControllableEntity().GetController();
        SingleThreadUppaalBot& testClass = dynamic_cast<SingleThreadUppaalBot&>(pcBot->GetControllableEntity().GetController());

        print_string(testClass.hurra);
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
}

void SingleThreadUppaalBot::ControlStep(){
    if(!stationPlan.empty() && isAtStation()){ // If we have a plan and we are at a point

        log_helper("Pre-reset");
        lastLocation = nextLocation;
        resetWaypointPlan();
        if(lastLocation == stationPlan.front()){ // Then we have reached the station @todo: Proper function for checking
            removeStationFromJobIfIn(lastLocation);
            resetStationPlan();
        }
        log_helper("Post-reset");
    }

    if(!hasJob()) {
        setJob();
    }

    if(hasJob() && stationPlan.empty()) //@todo: Have proper boolean function
    {
        log_helper("Constructs Station model");
        constructStationUppaalModel();
        log_helper("Constructed Station model");
        std::vector<int> stationPlan = getStationPlan(runStationModel());
        log_helper("Station plan has size " + std::to_string(stationPlan.size()));

        setStationPlan(stationPlan);
    }


    if(hasJob() && waypointPlan.empty()) //@todo: Have proper boolean function
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

    movementLogic();
}

void SingleThreadUppaalBot::resetWaypointPlan(){
    waypointPlan.clear();
}

void SingleThreadUppaalBot::resetStationPlan(){
    stationPlan.clear();
}

void SingleThreadUppaalBot::removeStationFromJobIfIn(int specificStation){
    if(std::find(job.begin(), job.end(), specificStation) != job.end()){
        job.erase(std::find(job.begin(), job.end(), specificStation));
    }

    log_helper("Job is now: ", false);
    for(int j : job){
        log_helper(std::to_string(j) + " ", false, false);
    }

    log_helper("", true, false);
}

bool SingleThreadUppaalBot::isAtStation(){
    const argos::CCI_PositioningSensor::SReading& tPosReads  = m_pcPosition->GetReading();
    Point nextPoint = sMap.getPointByID(nextLocation);

    if (Distance(tPosReads.Position,nextPoint) <= 0.15){
        argos::LOG<< m_strId <<" has arrived at station: "<< nextLocation <<std::endl;
        log_helper("Has arrived at station " + std::to_string(nextLocation));
        return true;
    }
    return false;
}

void SingleThreadUppaalBot::movementLogic(){
    const argos::CCI_PositioningSensor::SReading& tPosReads  = m_pcPosition->GetReading();

    Point nextPoint = sMap.getPointByID(nextLocation);
    argos::CRadians a,c,b;
    tPosReads.Orientation.ToEulerAngles(a,b,c);

    float oy = sin(a.GetValue());
    float ox = cos(a.GetValue());
    argos::CVector3 Ori(ox,oy,0);
    argos::CVector3 newOri = nextPoint - tPosReads.Position; // Direct Access to Map
    newOri.Normalize();

    double per = newOri.GetX()*Ori.GetY() - newOri.GetY()*Ori.GetX() ;
    double dotProd = newOri.GetX()*Ori.GetX() + newOri.GetY()*Ori.GetY();

    if(Distance(tPosReads.Position,nextPoint) <= 1.0 ){ // acceptance radius between point and robot
        controlStep(per,dotProd, Distance(tPosReads.Position, nextPoint)*60);
    }
    else {
        controlStep(per,dotProd, m_fWheelVelocity);
    }

}

void SingleThreadUppaalBot::controlStep(double per, double dotProd, float velocity){
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

        int numOfStations = sMap.stationIDs.size() + sMap.endStationIDs.size(); //Needs to be generalized to the current position in the given matrix.

        if(m[1] != "0" && stationsVisited.find(std::stoi(m[3])) == stationsVisited.end()){
            if(stationPlan.empty() && std::stoi(m[3]) == numOfStations) {
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
    this->stationPlan = stationPlan;
}

void SingleThreadUppaalBot::setWaypointPlan(std::vector<int> waypointPlan){
    this->waypointPlan = waypointPlan;
}

std::string SingleThreadUppaalBot::runStationModel(){
    std::string verifyta{"/home/martin/Desktop/uppaalStratego/bin-Linux/verifyta.bin"};
    //std::string verifyta{"~/phd/Uppaal/uppaal64-4.1.20-stratego-7/bin-Linux/verifyta"};
    std::string model{"./initial_model.xml"};

    std::string terminalCommand = verifyta + " " + model;

    std::string result;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    stream = popen(terminalCommand.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL) result.append(buffer);
        pclose(stream);
    }

    print_string(result);
    return result;
}

std::string SingleThreadUppaalBot::runWaypointModel(){
    std::string verifyta{"/home/martin/Desktop/uppaalStratego/bin-Linux/verifyta.bin"};
    std::string model{"./waypoint_model.xml"};

    std::string terminalCommand = verifyta + " " + model;

    std::string result;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    stream = popen(terminalCommand.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL) result.append(buffer);
        pclose(stream);
    }

    print_string(result);
    return result;
}

bool SingleThreadUppaalBot::hasJob() {
    return !job.empty();
}

void SingleThreadUppaalBot::setJob() {
    if(!hasJob()) {
        std::vector<int> nextJob = sMap.getNextJob();
        for(int j : nextJob)
            job.push_back(j);
    }

    log_helper("Job is now: ", false);
    for(int j : job){
        log_helper(std::to_string(j) + " ", false, false);
    }
    log_helper("", true, false);
}

std::vector<int> SingleThreadUppaalBot::getJob(){
    return job;
}

void SingleThreadUppaalBot::setNextLocation(int locationID){
    nextLocation = locationID;
}

void SingleThreadUppaalBot::constructStationUppaalModel(){
    std::ifstream partial_blueprint{std::string{std::filesystem::current_path()} + "/station_planning_blueprint.xml"};
    std::ofstream full_model{std::string{std::filesystem::current_path()} + "/initial_model.xml"};

    Robot self = sMap.getRobotByName(m_strId);

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
                         format_order(numOfStations, getJob()));
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

        pos = line.find("#XML_COMMENT_START#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(sMap.Robots, self) == 0)
                line.replace(pos, std::string{"#XML_COMMENT_START#"}.size(), "<!--");
            else
                line.replace(pos, std::string{"#XML_COMMENT_START#"}.size(), "");
        }

        pos = line.find("#XML_COMMENT_END#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(sMap.Robots, self) == 0)
                line.replace(pos, std::string{"#XML_COMMENT_END#"}.size(), "-->");
            else
                line.replace(pos, std::string{"#XML_COMMENT_END#"}.size(), "");
        }

        pos = line.find("#CODE_COMMENT_START#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(sMap.Robots, self) == 0)
                line.replace(pos, std::string{"#CODE_COMMENT_START#"}.size(), "/*");
            else
                line.replace(pos, std::string{"#CODE_COMMENT_START#"}.size(), "");
        }

        pos = line.find("#CODE_COMMENT_END#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(sMap.Robots, self) == 0)
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

        pos = line.find("#OTHER_CREATION#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(sMap.Robots, self) == 0)
                line.replace(pos, std::string{"#OTHER_CREATION#"}.size(), "");
            else {
                throw std::logic_error{"Not implemented yet"};
                line.replace(pos, std::string{"#OTHER_CREATION#"}.size(), "BLA");
            }
        }

        pos = line.find("#OTHER_IN_SYSTEM#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(sMap.Robots, self) == 0)
                line.replace(pos, std::string{"#OTHER_IN_SYSTEM#"}.size(), "");
            else {
                throw std::logic_error{"Not implemented yet"};
                line.replace(pos, std::string{"#OTHER_IN_SYSTEM#"}.size(), "BLA");
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

        full_model << line << std::endl;

    }

    full_model.close();
}

void SingleThreadUppaalBot::constructWaypointUppaalModel(){
    std::ifstream partial_blueprint{std::string{std::filesystem::current_path()} + "/station_planning_blueprint.xml"};
    std::ofstream waypoint_model{std::string{std::filesystem::current_path()} + "/waypoint_model.xml"};

    Robot self = sMap.getRobotByName(m_strId);

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
                         std::to_string(numOfOtherActiveRobots(sMap.Robots, self)));
        }


        std::vector<std::vector<float>> matrix = getDistanceMatrix(sMap);

        pos = line.find("#CUR_ORDER#");
        if (pos != std::string::npos) {
            std::vector<int> nextStation{};
            nextStation.push_back(stationPlan.front());
            line.replace(pos, std::string{"#CUR_ORDER#"}.size(),
                         format_order(matrix.size(), nextStation));
        }

        pos = line.find("#DISTANCE_MATRIX#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#DISTANCE_MATRIX#"}.size(), format_distance_matrix(matrix));
        }

        pos = line.find("#END_STATIONS#");
        if(pos != std::string::npos){
            line.replace(pos, std::string{"#END_STATIONS#"}.size(),
                         format_endstations(matrix.size(), sMap.endStationIDs));
        }

        pos = line.find("#XML_COMMENT_START#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(sMap.Robots, self) == 0)
                line.replace(pos, std::string{"#XML_COMMENT_START#"}.size(), "<!--");
            else
                line.replace(pos, std::string{"#XML_COMMENT_START#"}.size(), "");
        }

        pos = line.find("#XML_COMMENT_END#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(sMap.Robots, self) == 0)
                line.replace(pos, std::string{"#XML_COMMENT_END#"}.size(), "-->");
            else
                line.replace(pos, std::string{"#XML_COMMENT_END#"}.size(), "");
        }

        pos = line.find("#CODE_COMMENT_START#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(sMap.Robots, self) == 0)
                line.replace(pos, std::string{"#CODE_COMMENT_START#"}.size(), "/*");
            else
                line.replace(pos, std::string{"#CODE_COMMENT_START#"}.size(), "");
        }

        pos = line.find("#CODE_COMMENT_END#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(sMap.Robots, self) == 0)
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

        pos = line.find("#OTHER_CREATION#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(sMap.Robots, self) == 0)
                line.replace(pos, std::string{"#OTHER_CREATION#"}.size(), "");
            else {
                throw std::logic_error{"Not implemented yet"};
                line.replace(pos, std::string{"#OTHER_CREATION#"}.size(), "BLA");
            }
        }

        pos = line.find("#OTHER_IN_SYSTEM#");
        if(pos != std::string::npos){
            if(numOfOtherActiveRobots(sMap.Robots, self) == 0)
                line.replace(pos, std::string{"#OTHER_IN_SYSTEM#"}.size(), "");
            else {
                throw std::logic_error{"Not implemented yet"};
                line.replace(pos, std::string{"#OTHER_IN_SYSTEM#"}.size(), "BLA");
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

        waypoint_model << line << std::endl;

    }

    waypoint_model.close();
}

REGISTER_CONTROLLER(SingleThreadUppaalBot, "SingleThreadUppaalBot_controller")