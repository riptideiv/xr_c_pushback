#pragma once
#include "chassis.hpp"
#include "intake.hpp"
#include "odom.hpp"
#include "pid.hpp"
#include "pneumatics.hpp"
#include "utils.hpp"
namespace auton {
    void leftElims() {
        // get group of 3 balls
        pid::moveToPoint(0, 9.5, 1000, {.straight = true, .maxSpeed = 60, .earlyExitRange = 5});
        intk::intake(100);
        pid::swingToHeading(-45, 700, true, {.maxSpeed = 60});
        pid::driveDist(0.2, 0.2, 10);
        pid::driveWait(-0.1, -0.1, 150);
        chass::drive(0, 0);

        // go to match loader
        bot::setTopDescore(true);
        pid::turnToPoint(-28, 12.5, 500);
        intk::stop();
        pid::moveToPoint(-28, 12.5, 1500, {.earlyExitRange = 4});
        pid::driveWait(0.02, 0.02, 150);
        pid::turnToHeading(180, 700);

        // match load
        bot::toggleMatchLoader();
        pros::delay(250);
        intk::intake(100);
        pid::moveToPoint(-27.5, 2.27722, 1000, {.straight = true, .earlyExitRange = 2});
        pid::driveWait(0.2, 0.2, 800);

        // score in high goal
        pid::moveToPoint(-30.5, 35, 1000, {.forwards = false, .earlyExitRange = 3});
        pid::driveWait(-0.5, -0.5, 300);
        intk::scoreHigh(100);
        pid::turnToHeading(180, 700);
        for (int i = 0; i < 220 && !intk::wrongColorDetected(); i++) {
            pros::delay(10);
        } // 2.2 seconds max wait
        bot::toggleMatchLoader();
        intk::stop();

        // push high goal balls into control zone (turnAndDescore)
        utils::turnAndDescore();
        intk::scoreHigh(100);
        odom::setPose(0, 0, 0);
        pid::moveToPoint(0, 9, 700, {.straight = true});
        pid::driveWait(0.5, -0.3, 300);
    }
    void rightElims() { // get group of 3 balls
        pid::moveToPoint(0, 9.5, 1000, {.straight = true, .maxSpeed = 60, .earlyExitRange = 5});
        intk::intake(100);
        pid::swingToHeading(45, 700, false, {.maxSpeed = 60});
        pid::driveDist(0.2, 0.2, 10);
        pid::driveDist(0.25, 0.25, -2);

        // go to match loader
        bot::setTopDescore(true);
        // pid::swingToHeading(0, 1000, true, {.earlyExitRange = 10});
        pid::turnToPoint(32, 21.4577, 700);
        intk::stop();
        pid::moveToPoint(32, 21.4577, 1500, {.earlyExitRange = 4});
        pid::driveWait(0.02, 0.02, 150);
        pid::turnToHeading(180, 700);

        // match load
        bot::toggleMatchLoader();
        pros::delay(250);
        intk::intake(100);
        pid::moveToPoint(40.6962, 5.78805, 1000, {.straight = true, .earlyExitRange = 4});
        pid::driveWait(0.2, 0.2, 1050);

        // score in high goal
        pid::moveToPoint(39.8385, 35.8962, 1000, {.forwards = false, .earlyExitRange = 3});
        intk::scoreHigh(100);
        pid::turnToHeading(180, 700);
        for (int i = 0; i < 220 && !intk::wrongColorDetected(); i++) {
            pros::delay(10);
        } // 2.2 seconds max wait
        bot::toggleMatchLoader();
        intk::stop();

        // push high goal balls into control zone (turnAndDescore)
        utils::turnAndDescore();
        intk::scoreHigh(100);
        odom::setPose(0, 0, 0);
        pid::moveToPoint(0, 9, 700, {.straight = true});
        pid::driveWait(0.5, -0.3, 300);
    }
} // namespace auton