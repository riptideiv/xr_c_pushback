#include "riptide.hpp"
#include "intake.hpp"
#include "main.h"
#include "odom.hpp"
#include "pid.hpp"
#include "pneumatics.hpp"
#include "pros/misc.h"
#include "robot.hpp"
#include "utils.hpp"

namespace riptide {
    using bot::master;
    void initialize() {
        bot::initialize();
        odom::initialize();
    }
    void autonomous() {
        bot::toggleDescoreArm();
        // pid::driveTo(12, 2000);

        // // rightside preload only
        // pid::driveTo(33, 1500);
        // intk::scoreHigh(100);
        // pros::delay(300);
        // intk::stop();
        // pid::turnTo(-90, 1000);
        // pid::driveTo(20, 1000);
        // intk::scoreHigh(100);

        // rightside preload + match load
        intk::scoreHigh(100);
        pros::delay(300);
        intk::stop();
        pid::driveTo(32.5, 1500);
        bot::toggleMatchLoader();
        pid::turnTo(-90, 1000);
        intk::intake(100);
        pid::driveTo(-12, 1000);
        pros::delay(500);
        pid::driveTo(-13, 300, false);
        bot::toggleMatchLoader();
        pid::driveTo(20, 1000, false);
        intk::scoreHigh(100);

        // // leftside preload + match load
        // intk::scoreHigh(100);
        // pros::delay(300);
        // intk::stop();
        // pid::driveTo(34, 1500);
        // bot::toggleMatchLoader();
        // pid::turnTo(90, 1000);
        // intk::intake(100);
        // pid::driveTo(-15, 1000);
        // pros::delay(500);
        // pid::driveTo(-16, 300, false);
        // bot::toggleMatchLoader();
        // pid::driveTo(17, 1000, false);
        // intk::scoreHigh(100);
    }
    void disabled() {
    }
    void opcontrol() {
        // utils::runLateralPID_kPs(0, 10, 11, 2, 24, 2000);
        // utils::runLateralBSearchkP(0, 0, 10, 24, 2000); // 6.25
        // utils::runLateralBSearchkD(6.25, 0, 100, 24, 2000); // 75
        // utils::runAngularBSearchkP(0, 0, 10, 90, 1000); // 1.875
        // utils::runAngularBSearchkD(2.25, 0, 200, 90, 1000); // 143.75
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
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                bot::toggleDescoreArm();
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                bot::toggleMatchLoader();
            }
            chass::arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        }
    }
} // namespace riptide