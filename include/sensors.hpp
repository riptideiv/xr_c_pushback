#pragma once

#include "main.h"
namespace bot {
    extern pros::Imu imu1;
    extern pros::Imu imu2;
    extern pros::Rotation horizEnc, vertEnc;
    extern const double horizOffset, vertOffset; // horiz offset = rightward is +, vert offset = forward is +
    void initialize_sensors();
    void setRotation(double rotation);
    void setHeading(double heading);
    // Return corrected IMU rotation in degrees
    double getRotation();
} // namespace bot