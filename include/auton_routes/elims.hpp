#pragma once
#include "chassis.hpp"
#include "intake.hpp"
#include "odom.hpp"
#include "pid.hpp"
#include "pneumatics.hpp"
#include "utils.hpp"
namespace auton {
    void leftElims() {
        odom::setPose(-0.2, 3.22, 0);
        // get group of 3 balls
        pid::mv2pt(0, 12, 1000, {.straight = true, .maxSpeed = 60, .earlyExitRange = 5});
        intk::intake(100);
        pid::swing2hd(-45, 550, true, {.maxSpeed = 60});
        pid::mv2pt(-6.72568, 26.9864, 750, {.straight = true});

        // go to match loader
        bot::setTopDescore(true);
        pid::turn2pt(-30.5, 12.5, 500);
        intk::stop();
        pid::mv2pt(-30.5, 12.5, 1500, {.earlyExitRange = 15});
        pid::mv2pt(-30.5, 12.5, 1500, {.maxSpeed = 20, .earlyExitRange = 4});
        pid::driveWait(0.02, 0.02, 150);
        pid::turn2hd(180, 500);

        // match load
        bot::toggleMatchLoader();
        pros::delay(250);
        intk::intake(100);
        pid::mv2pt(-29, 0.27722, 1000, {.straight = true, .earlyExitRange = 3});
        pid::driveWait(0.25, 0.25, 800); // NEED TEST

        // score in high goal
        pid::mv2pt(-32.5, 33.5, 1000, {.forwards = false, .earlyExitRange = 3});
        pid::driveWait(-0.5, -0.5, 300);
        intk::scoreHigh(100);
        pid::turn2hd(180, 700);
        for (int i = 0; i < 200 && !intk::wrongColorDetected(); i++) {
            pros::delay(10);
        } // 2.2 seconds max wait
        bot::toggleMatchLoader();
        intk::stop();

        // // push high goal balls into control zone (turnAndDescore)
        // utils::turnAndDescore();
        // intk::scoreHigh(100);
        // odom::setPose(0, 0, 0);
        // pid::moveToPoint(0, 9, 350, {.straight = true});
        // pid::driveWait(0.4, -0.25, 300);

        // turnAndSetupDescoreBackward
        utils::turnAndSetupDescoreBackward();
        intk::outtake(100);
        odom::setPose(0, 0, 0);
        pid::driveDist(0.6, 0.6, -12);
        pid::driveWait(-0.4, 0.25, 300);
        pid::driveWait(-0.05, 0.05, 5000);
    }
    void rightElims() { // get group of 3 balls
        odom::setPose(-0.34, 4.02, 0);
        // get group of 3 balls
        pid::mv2pt(0, 12.0, 1000, {.straight = true, .maxSpeed = 60, .earlyExitRange = 4});
        intk::intake(100);
        pid::swing2hd(45, 700, false, {.maxSpeed = 60});
        pid::mv2pt(4.54353, 31, 750, {.straight = true});
        pid::driveDist(0.25, 0.25, -2);
        chass::drive(0, 0);
        odom::debugPrint();

        // go to match loader
        bot::setTopDescore(true);
        pid::turn2pt(34.7, 21, 700);
        intk::stop();
        pid::mv2pt(34.7, 21, 1500, {.earlyExitRange = 15});
        pid::mv2pt(34.7, 21, 1500, {.maxSpeed = 20, .earlyExitRange = 4});
        pid::driveWait(0.02, 0.02, 150);
        pid::turn2hd(180, 500);

        // match load
        bot::toggleMatchLoader();
        pros::delay(250);
        intk::intake(100);
        pid::mv2pt(40.5, 5, 1000, {.straight = true, .earlyExitRange = 4});
        pid::driveWait(0.25, 0.25, 800);

        // score in high goal
        bot::toggleMatchLoader();
        pid::mv2pt(40.8, 37, 1000, {.forwards = false, .earlyExitRange = 3});
        intk::scoreHigh(100);
        pid::turn2hd(180, 700);
        for (int i = 0; i < 200 && !intk::wrongColorDetected(); i++) {
            pros::delay(10);
        } // 2.2 seconds max wait
        bot::toggleMatchLoader();
        intk::stop();

        // // push high goal balls into control zone (turnAndDescore)
        // utils::turnAndDescore();
        // intk::scoreHigh(100);
        // odom::setPose(0, 0, 0);
        // pid::moveToPoint(0, 9, 350, {.straight = true});
        // pid::driveWait(0.4, -0.25, 300);

        // turnAndSetupDescoreBackward
        utils::turnAndSetupDescoreBackward();
        intk::outtake(100);
        odom::setPose(0, 0, 0);
        pid::driveDist(0.4, 0.4, -6);
        pid::driveDist(0.6, 0.6, -6);
        pid::driveWait(-0.4, 0.25, 300);
        pid::driveWait(-0.05, 0.05, 5000);
    }
} // namespace auton