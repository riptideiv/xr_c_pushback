#pragma once
#include "auton.hpp"
#include "chassis.hpp"
#include "intake.hpp"
#include "odom.hpp"
#include "pid.hpp"
#include "pneumatics.hpp"
#include "pros/rtos.hpp"
#include "utils.hpp"
namespace auton {
    void autonSkills() {
        intk::skills_midgoal = true;
        bot::setTopDescore(true);
        intk::intake(100);
        pid::driveWait(0.2, 0.2, 500);
        pid::driveWait(0.2, 0.15, 350);
        pid::driveWait(0.15, 0.2, 350);
        pid::driveWait(0.7, 0.7, 400);
        pid::driveWait(0.05, 0.05, 500);
        pid::driveWait(-0.2, -0.2, 300);
        pid::driveWait(0.2, 0.2, 200);
    }
} // namespace auton