#include "sensors.hpp"

namespace bot {
    pros::Imu imu(8);
    void initialize_sensors() {
        imu.reset(1);
    }
} // namespace bot