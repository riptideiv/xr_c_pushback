#pragma once

#include "main.h"
namespace bot {
    extern pros::Imu imu;
    extern pros::Rotation horizEnc, vertEnc;
    extern const double horizOffset, vertOffset; // horiz offset = rightward is +, vert offset = forward is +
    void initialize_sensors();
} // namespace bot