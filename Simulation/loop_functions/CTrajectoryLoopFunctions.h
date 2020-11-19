#ifndef CTrajectoryLoopFunctions_H
#define CTrajectoryLoopFunctions_H

#include "models/map/map_structure.hpp"
#include "../models/jobs/JobGenerator.hpp"

#include "argos3/core/simulator/loop_functions.h"
#include "argos3/plugins/robots/foot-bot/simulator/footbot_entity.h"
#include "argos3/plugins/simulator/entities/box_entity.h"



class CTrajectoryLoopFunctions : public argos::CLoopFunctions {

public:
    typedef std::map<argos::CFootBotEntity *, std::vector<argos::CVector3>> TWaypointMap;
    typedef std::vector<argos::CBoxEntity> BoxMap;

    TWaypointMap m_tWaypoints;
    BoxMap m_box_map;

public:
    virtual ~CTrajectoryLoopFunctions() {}

    virtual void Init(argos::TConfigurationNode &t_tree);

    inline const TWaypointMap &GetWaypoints() const { return m_tWaypoints; }

    inline const BoxMap &GetBoxMap() const { return m_box_map; }

private:
    std::shared_ptr<JobGenerator> jobGenerator;

    void setJobGenerator();
    void assignJobGeneratorToControllers();
};

#endif
