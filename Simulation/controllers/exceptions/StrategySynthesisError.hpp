

#ifndef SWARMSIMULATOR_STRATEGYSYNTHESISERROR_HPP
#define SWARMSIMULATOR_STRATEGYSYNTHESISERROR_HPP

#include <stdexcept>


class StrategySynthesisError : public std::runtime_error{

public:
    using std::runtime_error::runtime_error;
};


#endif //SWARMSIMULATOR_STRATEGYSYNTHESISERROR_HPP
