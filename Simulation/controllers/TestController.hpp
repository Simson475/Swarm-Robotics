#ifndef SWARMSIMULATOR_TESTCONTROLLER_HPP
#define SWARMSIMULATOR_TESTCONTROLLER_HPP

class TestController;

#include "SingleThreadBotGreedy.hpp"
#include "ExperimentData.hpp"
#include "Location.hpp"
#include "Action.hpp"

#include "Debugging.hpp"

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
    void setCurrentAction(Action action);
    void setPath(Path path);

protected:

    std::vector<int> constructStationPlan() override;
    std::vector<int> constructWaypointPlan() override;

    int agentId;
    Path path;

    void reachedPointEvent(int) override;
    void wait() override;
    Location currentLocation;
    Action currentAction;
    std::vector<int> getNextPointAndUpdateState();
    void updateCurrentLocation(Action currentAction);

    float waitClock;
    void startWaiting();

};

#endif