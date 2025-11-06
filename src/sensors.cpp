#include "sensors.hpp"
namespace bot {
    pros::Imu imu(1);
    void initialize_sensors() {
        imu.reset(1);
    }
} // namespace bot