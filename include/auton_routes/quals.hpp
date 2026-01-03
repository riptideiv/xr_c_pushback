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
        odom::setPose(-0.2, 3.22, 0);
        // get group of 3 balls
        pid::moveToPoint(0, 12, 1000, {.straight = true, .maxSpeed = 60, .earlyExitRange = 5});
        intk::intake(100);
        pid::swingToHeading(-45, 550, true, {.maxSpeed = 60});
        pid::moveToPoint(-6.72568, 26.9864, 750, {.straight = true});

        // score in mid goal
        pid::turnToPoint(14.2, 48.5, 500, {.forwards = false});
        intk::setPowers(-100, -5, 0, 0);
        pid::driveWait(-0.2, -0.2, 300);
        intk::stop();
        pid::moveToPoint(14.5, 48.2, 1000, {.forwards = false, .straight = true, .maxSpeed = 60, .earlyExitRange = 4});
        intk::scoringMid = true;
        pros::delay(100);
        intk::scoreMid(100);
        pid::driveWait(-0.1, -0.1, 600);
        intk::scoreMid(0);
        pid::driveWait(-0.1, -0.1, 200);
        intk::intake(100);

        // go to match loader
        bot::setTopDescore(true);
        pid::driveDist(0.2, 0.2, 1); // NOT NEEDED IN ELIMS VERSION (in case i forget)
        pid::turnToPoint(-28, 12.5, 500);
        intk::stop();
        pid::moveToPoint(-28, 12.5, 1500, {.earlyExitRange = 15});
        pid::moveToPoint(-28, 12.5, 1500, {.maxSpeed = 20, .earlyExitRange = 4});
        pid::driveWait(0.02, 0.02, 150);
        pid::turnToHeading(180, 500);

        // match load
        bot::toggleMatchLoader();
        pros::delay(250);
        intk::intake(100);
        pid::moveToPoint(-29, 0.27722, 1000, {.straight = true, .earlyExitRange = 2});
        pid::driveWait(0.4, 0.4, 800); // NEED TEST

        // score in high goal
        pid::moveToPoint(-32.5, 33.5, 1000, {.forwards = false, .earlyExitRange = 3});
        pid::driveWait(-0.5, -0.5, 300);
        intk::scoreHigh(100);
        pid::turnToHeading(180, 700);
        for (int i = 0; i < 190 && !intk::wrongColorDetected(); i++) {
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
        pid::moveToPoint(0, -20, 1000, {.forwards = false, .straight = true, .earlyExitRange = 3});
        pid::driveWait(-0.4, 0.25, 300);
        pid::driveWait(-0.05, 0.05, 5000);
    }
    void rightQuals() {
        odom::setPose(-0.34, 4.02, 0);
        // get group of 3 balls
        pid::moveToPoint(0, 11.5, 1000, {.straight = true, .maxSpeed = 60, .earlyExitRange = 5});
        intk::intake(100);
        pid::swingToHeading(45, 700, false, {.maxSpeed = 60});
        pid::moveToPoint(4.54353, 30, 750, {.straight = true});
        pid::driveDist(0.25, 0.25, -2);
        chass::drive(0, 0);
        odom::debugPrint();

        // score in low goal
        pid::turnToPoint(-1.7, 41.5, 500);
        intk::setPowers(-100, 5, 0, 0);
        pid::moveToPoint(-1.7, 41.5, 1000, {.straight = true, .maxSpeed = 60, .earlyExitRange = 4});
        intk::outtake(100);
        pid::driveWait(0.05, -0.05, 600);
        pid::driveWait(-0.2, -0.2, 400);
        intk::intake(100);

        // go to match loader
        bot::setTopDescore(true);
        pid::swingToHeading(0, 1000, true, {.earlyExitRange = 10}); // NOT NEEDED IN ELIMS VERSION (in case i forget to remove after copy pasting)
        pid::turnToPoint(37, 21, 700);
        intk::stop();
        pid::moveToPoint(37, 21, 1500, {.earlyExitRange = 15});
        pid::moveToPoint(37, 21, 1500, {.maxSpeed = 20, .earlyExitRange = 4});
        pid::driveWait(0.02, 0.02, 150);
        pid::turnToHeading(180, 500);

        // match load
        bot::toggleMatchLoader();
        pros::delay(250);
        intk::intake(100);
        pid::moveToPoint(42, 5, 1000, {.straight = true, .earlyExitRange = 4});
        pid::driveWait(0.4, 0.4, 800);

        // score in high goal
        pid::moveToPoint(41.3385, 37, 1000, {.forwards = false, .earlyExitRange = 3});
        intk::scoreHigh(100);
        pid::turnToHeading(180, 700);
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
        pid::moveToPoint(0, -20, 1000, {.forwards = false, .straight = true, .minSpeed = 30, .earlyExitRange = 5});
        pid::driveWait(-0.4, 0.25, 300);
        pid::driveWait(-0.05, 0.05, 5000);
    }
} // namespace auton