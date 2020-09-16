#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <pthread.h>

//using namespace std;
/* Include the controller definition */
#include <CFootBotDiffusion.h>
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

//#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
int CFootBotDiffusion::counter = 0;
#define NUM_THREADS 20
pthread_t threads[NUM_THREADS];
std::vector<arg_struct> args(NUM_THREADS);
bool paused = false;
void *status;
/****************************************/
/****************************************/
CFootBotDiffusion::CFootBotDiffusion():
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcPosition(NULL),
    m_cAlpha(7.5f),
    m_fDelta(0.1f),
    m_fWheelVelocity(VELOCITY),
    m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                            ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
    std::ofstream outfile;
    outfile.open("test.txt", std::ios_base::app); // append instead of overwrite
    outfile<<Map_Structure::get_instance().Robots.size()<<"SIZE"<<std::endl;
    for (auto& robots: sMap.Robots){
        outfile<<robots.getInitialLoc().getId()<<std::endl;
    }
    outfile<<"POINTS"<<std::endl;
    for (auto& point: Map_Structure::get_instance().points){

        outfile<<point.getId()<<std::endl;
    }
    outfile.close();
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
    m_id          = GetId();
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
    m_pcPosition  = GetSensor  <CCI_PositioningSensor>("positioning");
    /*
     * Parse the configuration file
     *
     * The user defines this part. Here, the algorithm accepts three
     * parameters and it's nice to put them in the config file so we don't
     * have to recompile if we want to try other settings.
     */
    GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
    m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
    GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

    //CQTOpenGLUserFunctions().GetQTOpenGLWidget().PauseExperiment(); function to pause experiment
}

/****************************************/
/****************************************/
void CFootBotDiffusion::getShortestPath(int n, bool stations){

    if(!stations){
        sMap.Robots[n].clearWaypoints();
        sMap.Robots[n].addWaypoints(sMap.findPath(sMap.Robots[n].getCurrentID().getId(),sMap.Robots[n].getRemainingStations().front().getId()));
        //pthread_detach(threads[n+10]);
        pthread_cancel(threads[n+10]);
    }else { pthread_detach(threads[n]);
        sMap.Robots[n].changeStatus(Status::waitWaypoints);
        return;
    }
    if(sMap.Robots[n].getRemainingWaypoints().size()<=1){
        sMap.Robots[n].changeStatus(Status::requestStations);
    }else sMap.Robots[n].changeStatus(Status::requestWaypoints);

}

void CFootBotDiffusion::ControlStep() {
    Map_Structure &sMap = Map_Structure::get_instance();
    int n = sMap.getRobotById(m_id);
    //if(!paused || sMap.Robots[n].getStatus() == Status::waitStations  ||sMap.Robots[n].getStatus() == Status::waitWaypoints){

    std::ofstream outfile;
    outfile.open("test.txt", std::ios_base::app); // append instead of overwrite
    outfile<< " CONTROL STEPS " <<std::endl;
    outfile<<Map_Structure::get_instance().Robots.size()<<"SIZE"<<std::endl;
    for (auto& robots: sMap.Robots){
        outfile<<robots.getInitialLoc().getId()<<std::endl;
    }
    outfile<<"POINTS"<<std::endl;
    for (auto& point: Map_Structure::get_instance().points){

        outfile<<point.getId()<<std::endl;
    }
    outfile<< " CONTROL STEPS " <<std::endl;
    outfile.close();
    switch(sMap.Robots[n].getStatus()){
        case Status::available : if(!lookForJob(n)){ // if out of jobs
                if(Distance(sMap.Robots[n].getfootBot()->GetEmbodiedEntity().GetOriginAnchor().Position
                    ,sMap.Robots[n].getInitialLoc()) >= 0.20){movementLogic(n);} // move back to initial
                else {m_pcWheels->SetLinearVelocity(0.0f, 0.0f);}} // if we are in initial and out of jobs we stand still

            break;
        case Status::requestStations : if( sMap.Robots[n].getRemainingStations().size()>2){
                createUppaalTask(n, "Stations",n, true);

            }
            createUppaalTask(n, "Waypoints", n+10,true); // whenever we request for stations we also need to request for waypoints
            sMap.Robots[n].changeStatus(Status::occupied);
            break;
        case Status::waitStations :    sMap.totalTries++;
            if(0 == pthread_tryjoin_np(threads[n], &status)){//(void**)0)){
                extractUppaalTask(n, "Stations",n);
                m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
                sMap.Robots[n].changeStatus(Status::waitWaypoints);}
            else{ sMap.timesUppaalFailed++;
                argos::LOGERR<<m_id<<" (Stations)Uppaal was too slow from "
                             <<sMap.getPointByID(sMap.Robots[n].getPreviousLoc()).getName()<<" to "
                             <<sMap.getPointByID(sMap.Robots[n].getCurrentID().getId()).getName()
                             <<std::endl<<"Uppaal fail ratio"<<sMap.timesUppaalFailed<<"/"<<sMap.totalTries<<std::endl;
                getShortestPath(n, true);}
            break;
        case Status::requestWaypoints : createUppaalTask(n, "Waypoints", n+10,false);
            sMap.Robots[n].changeStatus(Status::occupied);
            break;
        case Status::waitWaypoints :sMap.totalTries++;
            if(0 == pthread_tryjoin_np(threads[n+10],&status)){ //(void**)0)){
                extractUppaalTask(n, "Waypoints", n+10);
                if(sMap.Robots[n].getRemainingWaypoints().size()<=1 ){
                    sMap.Robots[n].changeStatus(Status::requestStations);
                }else sMap.Robots[n].changeStatus(Status::requestWaypoints);}
            else{sMap.timesUppaalFailed++;
                argos::LOGERR<<m_id<<" (Waypoints)Uppaal was too slow from "
                             <<sMap.getPointByID(sMap.Robots[n].getPreviousLoc()).getName()<<" to "
                             <<sMap.getPointByID(sMap.Robots[n].getCurrentID().getId()).getName()
                             <<std::endl<<"Uppaal fail ratio: "<<sMap.timesUppaalFailed<<"/"<<sMap.totalTries<<std::endl;
                getShortestPath(n, false);}
            break;
        case Status::occupied : movementLogic(n);
            break;
    }
    //}else{m_pcWheels->SetLinearVelocity(0.0f, 0.0f);}
}


void CFootBotDiffusion::Reset(){
}

static void* callStratego(void *arguments){
    struct arg_struct *args = (struct arg_struct *) arguments;
    std::string mystring=std::string("./uppaalStratego/callStratego.exe ")+args -> id+" "+ args->choice;
    connectStratego(args->choice, args->id, args->dynamic, args->path);

    /*string action;
    FILE* stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    stream = popen(mystring.c_str(), "r");
    if (stream) {
    while (!feof(stream))
       if (fgets(buffer, max_buffer, stream) != NULL) action.append(buffer);
          pclose(stream);
    }
     args->result=action;*/
    pthread_exit(NULL);
}

void printJob(std::vector<Robot> robots){
    for(auto& r: robots){
        std::cout<<"Robot id: " << r.getfootBot()->GetId();
        for(auto& s : r.getRemainingStations()){
            std::cout<<"Stations ID: "<<s.getId()<<std::endl;
        }
    }
    for(auto& r: robots){
        for(auto& w : r.getRemainingWaypoints()){
            std::cout<<"Waypoint ID: "<<w.getId()<<std::endl;
        }
    }
}

void CFootBotDiffusion::createUppaalTask(int n, std::string choice, int threadNr, bool stations){
    std::string dynamic= sMap.Robots[n].createDynamicJson(sMap.Robots, n, stations);
    args[threadNr].id = m_id;
    args[threadNr].choice = choice;
    args[threadNr].dynamic = dynamic;
    args[threadNr].path = sMap.folderPath;
    pthread_create(&threads[threadNr], NULL, callStratego, &args[threadNr]);
    std::cout <<args[threadNr].id<<" Calls UPPAAL "<<choice<<std::endl;
    paused = true;
}

void CFootBotDiffusion::extractUppaalTask(int n, std::string choice, int threadNr){
    //std::cout <<m_id<<" has received plan"<<endl;
    sMap.Robots[n].converJSONStation(m_id, choice);
    if(choice == "Stations"){
        sMap.Robots[n].setRemainingStations(sMap.points);
        //sMap.Robots[n].setCurrStationTarget();
    }
    if(choice == "Waypoints"){
        sMap.Robots[n].setRemainingWaypoints(sMap.points);
    }
    paused = false;
    // printJob(sMap.Robots);
}

void CFootBotDiffusion::movementLogic(int n){
    const CCI_PositioningSensor::SReading&       tPosReads  = m_pcPosition->GetReading();
    if (sMap.Robots[n].getWatch() == -1){ //check if the robot is suppose to stay still
        Point* currTarget;
        if(sMap.Robots[n].getStatus() == Status::available){
            currTarget = &sMap.Robots[n].getInitialLoc();
            //currTarget = *p;
        }
        else currTarget = sMap.Robots[n].getNextWayPoint();  // determine the next target location of the robot
        //std::cout<<currTarget.getName()<<std::endl;
        //if(sMap.Robots[n].getStatus() == Status::occupied && sMap.Robots[n].getWatch() == -1)

        CRadians a,c,b;
        tPosReads.Orientation.ToEulerAngles(a,b,c);

        float oy = sin(a.GetValue());
        float ox = cos(a.GetValue());
        CVector3 Ori(ox,oy,0);
        CVector3 newOri = *currTarget - tPosReads.Position;
        newOri.Normalize();

        double per = newOri.GetX()*Ori.GetY() - newOri.GetY()*Ori.GetX() ;
        double dotProd = newOri.GetX()*Ori.GetX() + newOri.GetY()*Ori.GetY();
//std::cout<< m_id << " my destination"<<currTarget->getName()<<" "<< Distance(tPosReads.Position,*currTarget)<<std::endl;
        if(Distance(tPosReads.Position,*currTarget) <= 1.5 ){//&& currTarget->getId() != sMap.Robots[n].getCurrentID().getId()){ // acceptance radius between point and robot
            if(!sMap.getPointByID(sMap.Robots[n].getCurrentTarget()->getId()).isOccupied()){
                if(Distance(tPosReads.Position,*currTarget) <= 0.15){
                    sMap.Robots[n].setPreviousLoc(sMap.Robots[n].getCurrentID().getId());
                    sMap.Robots[n].updateCurrent(&sMap.getPointByID(currTarget->getId()));
                    m_pcWheels->SetLinearVelocity(0.0f, 0.0f); // setting robots speed to 0 when the target is reached
                    switch(currTarget->getType()){

                        case Type::station : sMap.getPointByID(sMap.Robots[n].getCurrentTarget()->getId()).setOccupied(true);
                            std::cout <<"------------------"<< sMap.Robots[n].getCurrentTarget()->getId() << "set to true "<<std::endl;
                            sMap.Robots[n].increment(sMap.Robots[n].getWatch()+1);
                            if(sMap.Robots[n].getRemainingStations().size()<3){
                                sMap.Robots[n].changeStatus(Status::waitWaypoints);
                            }else
                                sMap.Robots[n].changeStatus(Status::waitStations);
                            LOG<<m_id <<" is loading items in station: "<<sMap.Robots[n].getRemainingStations().front().getName()<<std::endl;
                            sMap.Robots[n].removeFirstStation();
                            break;
                        case Type::endpoint : //sMap.getPointByID(sMap.Robots[n].getCurrentTarget()->getId()).setOccupied(true);
                            //std::cout <<"------------------"<< sMap.Robots[n].getCurrentTarget()->getId() << "set to true "<<std::endl;
                            sMap.Robots[n].cleanJob();
                            sMap.Robots[n].changeStatus(Status::available);
                            std::cout<<sMap.Robots[n].getfootBot()->GetId()<<": <Find new job or go to initial>"<<std::endl;
                            sMap.Robots[n].increment(sMap.Robots[n].getWatch()+1);
                            break;
                            //case for the charging station *Future work*
                        default : sMap.Robots[n].changeStatus(Status::waitWaypoints);
                    }
                }else {if(m_id == "fzb1")LOGERR<<Distance(tPosReads.Position,*currTarget)*60<<std::endl; controlStep(per,dotProd,  Distance(tPosReads.Position,*currTarget)*60);}
            }else {LOGERR<<sMap.Robots[n].getCurrentTarget()->getId()<<"OCCUPIED WAITING"<<std::endl;m_pcWheels->SetLinearVelocity(0, 0);}

        }else{ controlStep(per,dotProd, m_fWheelVelocity);}

    }else {sMap.Robots[n].increment(sMap.Robots[n].getWatch()+1);m_pcWheels->SetLinearVelocity(0, 0);}
    if (sMap.Robots[n].getWatch() == stationDelay){
        sMap.getPointByID(sMap.Robots[n].getCurrentTarget()->getId()).setOccupied(false);
        std::cout <<"------------------"<< sMap.Robots[n].getCurrentTarget()->getId() << "set to false "<<std::endl;
        sMap.Robots[n].setCurrStationTarget();
        std::cout<<sMap.Robots[n].getfootBot()->GetId()<<": finished loading items" << std::endl;
        sMap.Robots[n].increment(-1);
    }
}

void CFootBotDiffusion::controlStep(double per, double dotProd, float velocity){
    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    CVector2 cAccumulator;
    for(size_t i = 0; i < tProxReads.size(); ++i){
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator /= tProxReads.size();
    /* If the angle of the vector is small enough and the closest obstacle
     * is far enough, continue going straight, otherwise curve a little
    */
    argos::Real turnRate;
    if(per>0.5 || per<-0.5) {turnRate = 10.0f;} else {turnRate = 3.0f;}// while the angle is big our turn rate is fast
    if(per<0.1 && per>-0.1) {turnRate = 1.0f;} //if the angle is small then our turn rate is reduced to 1
    CRadians cAngle = cAccumulator.Angle();
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

/****************************************/
/****************************************/
bool CFootBotDiffusion::lookForJob(int n){
    if(sMap.jobs.empty()){return false;}
    plotData();
    sMap.Robots[n].clearStations();
    sMap.Robots[n].clearWaypoints();
    sMap.Robots[n].cleanJob();
    for(auto& j: sMap.jobs.front()){
        sMap.Robots[n].addSinglePickUp(sMap.getPointByID(j));
    }
    sMap.jobs.erase(sMap.jobs.begin());
    sMap.Robots[n].sortJob(sMap.shortestDistances);
    std::cout << sMap.Robots[n].getCurrentID().getId() << "            " << sMap.Robots[n].getJob().front().getId() <<std::endl;
    sMap.Robots[n].addWaypoints(sMap.findPath(sMap.Robots[n].getCurrentID().getId(),sMap.Robots[n].getJob().front().getId()));
    sMap.Robots[n].setCurrStationTarget();
    if(sMap.Robots[n].getRemainingWaypoints().size()<=1){
        sMap.Robots[n].changeStatus(Status::requestStations);
    }else sMap.Robots[n].changeStatus(Status::requestWaypoints);
    return true;

}

void CFootBotDiffusion::plotData(){
    counter++;
    std::ofstream outfile;

    outfile.open("Data.txt", std::ios_base::app); // append instead of overwrite
    outfile << counter << " " << argos::CSimulator::GetInstance().GetSpace().GetSimulationClock()<<"\n";// << " " << std::endl;

}

REGISTER_CONTROLLER(CFootBotDiffusion, "CFootBotDiffusion_controller")
