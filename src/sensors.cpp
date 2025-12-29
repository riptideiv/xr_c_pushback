#include "sensors.hpp"

namespace bot {
    pros::Imu imu(20);
    pros::Rotation horizEnc(13), vertEnc(14);
    const double horizOffset = -2.55036, vertOffset = 2.04396; // horiz offset = rightward is +, vert offset = forward is +
    void initialize_sensors() {
        imu.reset(1);
        horizEnc.reset();
        vertEnc.reset();
    }
} // namespace bot