#ifndef SWARMSIMULATOR_TESTCONTROLLER_HPP
#define SWARMSIMULATOR_TESTCONTROLLER_HPP

class TestController;

#include "SingleThreadBotGreedy.hpp"
#include "ExperimentData.hpp"
#include "Location.hpp"

class TestController : public SingleThreadBotGreedy {

public:
    /* Class constructors. */
    using SingleThreadBotGreedy::SingleThreadBotGreedy;
    TestController(const TestController&) = default;
    TestController(TestController&&) noexcept = default;

    /* Class Variables */
    std::vector<int> receivedWaypointPlan;

    /* Properties */
    int getAgentId();
    void setAgentId(int id);
    Location getCurrentLocation();
    Action getCurrentAction();

    /* Methods */
    std::vector<Point> findOptimalPath();

protected:

    std::vector<int> constructStationPlan() override;
    std::vector<int> constructWaypointPlan() override;

    int agentId;
    Path path;

    void reachedPointEvent(int) override;
    Location currentLocation;
    Action currentAction;
    std::vector<int> getNextPointAndUpdateState();

};

#endif