#include "sensors.hpp"

namespace bot {
    pros::Imu imu(20);
    pros::Rotation horizEnc(13), vertEnc(14);
    const double horizOffset = 4.13559, vertOffset = -0.967106; // horiz offset = rightward is +, vert offset = forward is +
    // Correction factor to map IMU-reported rotation to actual rotation.
    // Example: sensor reads 358 when actual is 360 -> correction = 360.0/358.0
    static const double imuRotationCorrection = 360.0 / 358.0;

    void initialize_sensors() {
        imu.reset(1);
        horizEnc.reset();
        vertEnc.reset();
    }

    // Return corrected IMU rotation in degrees
    double getRotation() {
        return imu.get_rotation() * imuRotationCorrection;
    }
} // namespace bot