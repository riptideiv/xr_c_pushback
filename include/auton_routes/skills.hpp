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
        utils::turnAndSetupDescoreBackward();
        intk::outtake(100);
        odom::setPose(0, 0, 0);
        pid::driveDist(0.6, 0.6, -12);
        pid::driveWait(-0.4, 0.25, 300);
        pid::driveWait(-0.05, 0.05, 5000);
    }
} // namespace auton