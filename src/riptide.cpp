#include "riptide.hpp"
#include "main.h"
#include "odom.hpp"
#include "robot.hpp"

namespace riptide {
    using bot::master;
    void initialize() {
        bot::initialize();
        odom::initialize();
    }
    void autonomous() {
        odom::pdrive(12, 1000);
    }
    void disabled() {
    }
    void opcontrol() {
        while (1) {
            pros::delay(10);
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                intk::intake(100);
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                intk::outtake(100);
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intk::scoreHigh(100);
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intk::scoreMid(100);
            } else {
                intk::stop();
            }
            chass::arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        }
    }
} // namespace riptide