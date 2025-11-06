#include "robot.hpp"

namespace bot {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    void initialize() {
        chass::initialize();
        initialize_sensors();
        intk::initialize();
    }
} // namespace bot