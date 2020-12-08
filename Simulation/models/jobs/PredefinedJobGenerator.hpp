#ifndef SWARMSIMULATOR_PREDEFINEDJOBGENERATOR_HPP
#define SWARMSIMULATOR_PREDEFINEDJOBGENERATOR_HPP

#include "JobGenerator.hpp"
#include "Job.hpp"

#include <random>

class PredefinedJobGenerator : public JobGenerator  {
public:
    PredefinedJobGenerator(int numOfStations, std::set<int> endStations, int numOfJobs);
    virtual std::unique_ptr<Job> getNextJob() override;

protected:
    virtual std::unique_ptr<Job> generateJob() override;

private:
    std::mt19937 eng; //Using the same seed all the time.
    std::uniform_int_distribution<> distr;
    std::uniform_int_distribution<> distrEnd;
};


#endif //SWARMSIMULATOR_PREDEFINEDJOBGENERATOR_HPP
