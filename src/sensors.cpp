#include "sensors.hpp"

namespace bot {
    pros::Imu imu(8);
    pros::Rotation horizEnc(11), vertEnc(14);
    const double horizOffset = 1.0, vertOffset = 2.4; // horiz offset = rightward is +, vert offset = forward is +
    void initialize_sensors() {
        imu.reset(1);
    }
} // namespace bot