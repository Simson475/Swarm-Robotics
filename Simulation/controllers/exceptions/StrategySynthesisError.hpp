

#ifndef SWARMSIMULATOR_STRATEGYSYNTHESISERROR_HPP
#define SWARMSIMULATOR_STRATEGYSYNTHESISERROR_HPP

#include <stdexcept>


class StrategySynthesisError : public std::runtime_error{

public:
    explicit StrategySynthesisError(std::string msg);
};


#endif //SWARMSIMULATOR_STRATEGYSYNTHESISERROR_HPP
