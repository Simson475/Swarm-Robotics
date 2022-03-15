#ifndef SWARMSIMULATOR_TESTCONTROLLER_HPP
#define SWARMSIMULATOR_TESTCONTROLLER_HPP

class TestController;

#include "SingleThreadBotGreedy.hpp"
#include "HighLevelCBS.hpp"

class TestController : public SingleThreadBotGreedy {

public:
    /* Class constructors. */
    using SingleThreadBotGreedy::SingleThreadBotGreedy;
    TestController(const TestController&) = default;
    TestController(TestController&&) noexcept = default;

    /* Class Variables */
    std::vector<int> receivedWaypointPlan;

    /* Methods */
    std::vector<Point> findOptimalPath();

protected:

    std::vector<int> constructStationPlan() override;
    std::vector<int> constructWaypointPlan() override;

};

#endif