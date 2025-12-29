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
    void leftQuals() {
        // get group of 3 balls
        pid::moveToPoint(0, 9.5, 1000, {.straight = true, .maxSpeed = 60, .earlyExitRange = 5});
        intk::intake(100);
        pid::swingToHeading(-45, 700, true, {.maxSpeed = 60});
        pid::driveDist(0.2, 0.2, 10);
        pid::driveWait(-0.1, -0.1, 150);
        chass::drive(0, 0);

        // score in mid goal
        pid::turnToPoint(14.5, 48.2, 500, {.forwards = false});
        intk::setPowers(-100, -5, 0, 0);
        pid::driveWait(-0.2, -0.2, 300);
        intk::stop();
        pid::moveToPoint(14.5, 48.2, 1000, {.forwards = false, .straight = true, .maxSpeed = 60, .earlyExitRange = 4});
        intk::scoringMid = true;
        pros::delay(100);
        intk::scoreMid(100);
        pid::driveWait(-0.1, -0.1, 300);
        intk::scoreMid(0);
        pid::driveWait(-0.1, -0.1, 200);
        intk::intake(100);

        // go to match loader
        bot::setTopDescore(true);
        pid::turnToPoint(-27, 12.5, 500);
        intk::stop();
        pid::moveToPoint(-27, 12.5, 1500, {.earlyExitRange = 4});
        pid::driveWait(0.02, 0.02, 150);
        pid::turnToHeading(180, 700);

        // match load
        bot::toggleMatchLoader();
        pros::delay(250);
        intk::intake(100);
        pid::moveToPoint(-26.5, 2.27722, 1000, {.straight = true, .earlyExitRange = 2});
        pid::driveWait(0.2, 0.2, 800);

        // score in high goal
        pid::moveToPoint(-29.5, 35, 1000, {.forwards = false, .earlyExitRange = 3});
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
    void rightQuals() {
        // get group of 3 balls
        pid::moveToPoint(0, 9.5, 1000, {.straight = true, .maxSpeed = 60, .earlyExitRange = 5});
        intk::intake(100);
        pid::swingToHeading(45, 700, false, {.maxSpeed = 60});
        pid::driveDist(0.2, 0.2, 10);
        pid::driveDist(0.25, 0.25, -2);

        // score in low goal
        pid::turnToPoint(-0.5, 42, 500);
        intk::setPowers(-100, 5, 0, 0);
        pid::moveToPoint(-0.5, 42, 1000, {.straight = true, .maxSpeed = 60, .earlyExitRange = 4});
        intk::outtake(80);
        pid::driveWait(0.05, -0.05, 300);
        pid::driveWait(-0.2, -0.2, 500);
        intk::intake(100);

        // go to match loader
        bot::setTopDescore(true);
        pid::swingToHeading(0, 1000, true, {.earlyExitRange = 10}); // NOT NEEDED IN ELIMS VERSION (in case i forget to remove after copy pasting)
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