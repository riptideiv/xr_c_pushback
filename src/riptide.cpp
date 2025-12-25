#include "riptide.hpp"
#include "auton.hpp"
#include "display.hpp"
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
        lvgl_init();
        bot::initialize();
        odom::initialize();
        auton::initialize();
        display::initialize();
    }
    void autonomous() {
        auton::runSelectedAuton();
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
            if (auton::auton_ran == false)
                continue;
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
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                bot::toggleTopDescore();
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                bot::toggleMiddleDescore();
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                bot::toggleMatchLoader();
            }

            chass::arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        }
    }
} // namespace riptide