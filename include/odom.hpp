#pragma once

#include "main.h"
namespace odom {
    extern double xpos, ypos, xvelo, yvelo, speed, angvelo;
    extern pros::Task *odomTask;
    void odomLoop();
    void initialize();
    void pdrive(int targ, int t);
} // namespace odom