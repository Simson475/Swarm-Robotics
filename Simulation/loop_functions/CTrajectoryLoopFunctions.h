#ifndef CTrajectoryLoopFunctions_H
#define CTrajectoryLoopFunctions_H

#include "models/map/map_structure.hpp"
#include "../models/jobs/JobGenerator.hpp"
#include "../models/jobs/PredefinedJobGenerator.hpp"
#include "../models/jobs/PredefinedDescreteJobGenerator.hpp"

#include "argos3/core/simulator/loop_functions.h"
#include "argos3/plugins/robots/foot-bot/simulator/footbot_entity.h"
#include "argos3/plugins/simulator/entities/box_entity.h"



class CTrajectoryLoopFunctions : public argos::CLoopFunctions {

public:
    virtual ~CTrajectoryLoopFunctions() = default;

    inline void Init(argos::TConfigurationNode &t_tree) override;

    bool IsExperimentFinished() override;

    void PreStep() override;

    void PostExperiment() override;

private:
    std::shared_ptr<JobGenerator> jobGenerator;

    void initJobGenerator();
    void setRobotFolders();
    void assignJobGeneratorToControllers();
    void removeOldLogFile();
    void setInitLocationOnControllers(Map_Structure&);
    void haveControllersAccessEachOther();

    void setUpDataCollection();
};

#endif
