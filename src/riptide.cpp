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
        display::initialize();
        master.rumble("-");
    }
    void autonomous() {
        auton::runSelectedAuton();
    }
    void disabled() {
        while (1) {
            pros::delay(100);
            auton::auton_ran = true;
        }
    }
    bool opControlActive = false;
    void opcontrol() {
        // utils::runLateralPID_kPs(0, 10, 11, 2, 24, 2000);
        // utils::runLateralBSearchkP(0, 0, 10, 24, 2000);
        // utils::runLateralBSearchkD(6, 0, 500, 48, 2000); // P=6, D=250
        // utils::runAngularBSearchkP(0, 0, 10, 90, 1000);
        // utils::runAngularBSearchkD(6, 0, 2000, 90, 1000); // (6,375)
        auton::initialize();
        while (1) {
            pros::delay(10);
            if (auton::auton_ran == false) {
                opControlActive = false;
                continue;
            }
            // if (opControlActive == false) {
            //     bot::setTopDescore(true);
            //     opControlActive = true;
            // }

            // intake control
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                intk::intake(100);
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
                    intk::outtake(60);
                else
                    intk::outtake(100);
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intk::scoreHigh(100);
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intk::scoreMid(100);
            } else {
                intk::stop();
            }

            // pneumatic control
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                bot::toggleTopDescore();
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                bot::toggleMiddleDescore();
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                bot::toggleMatchLoader();
            }

            // macros
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                utils::turnAndSetupDescoreBackward();
            }

            // debugPrint
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                odom::debugPrint();
            }
            chass::arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        }
    }
} // namespace riptide