

#ifndef SWARMSIMULATOR_PREDEFINEDDESCRETEJOBGENERATOR_HPP
#define SWARMSIMULATOR_PREDEFINEDDESCRETEJOBGENERATOR_HPP

#include "JobGenerator.hpp"
#include "Job.hpp"

#include <random>

class PredefinedDescreteJobGenerator : public JobGenerator  {
public:
    PredefinedDescreteJobGenerator(int numOfStations, std::set<int> endStations, int numOfJobs);
    std::unique_ptr<Job> getNextJob() override;

protected:
    std::unique_ptr<Job> generateJob() override;

private:
    std::mt19937 eng; //Using the same seed all the time.
    std::discrete_distribution<int>distr;
    std::uniform_int_distribution<> distrEnd;
};


#endif //SWARMSIMULATOR_PREDEFINEDDESCRETEJOBGENERATOR_HPP
