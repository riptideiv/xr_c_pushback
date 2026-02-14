#pragma once
#include "auton.hpp"
#include "chassis.hpp"
#include "intake.hpp"
#include "odom.hpp"
#include "pid.hpp"
#include "pneumatics.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "sensors.hpp"
#include "utils.hpp"
namespace auton {
    void soloAWP() {
        odom::setPose(-1.2, -1, bot::getRotation());
        // push alliance
        pid::mv2pt(-1.2, -6.43803, 1000, {.forwards = false, .earlyExitRange = 3});

        // matchload1
        pid::mv2pt(1.40939, 47, 1350);
        pid::turn2hd(90, 550);
        bot::setMatchLoader(true);
        pid::turn2hd(90, 250);
        intk::intake(100);
        pid::mv2pt(16.5, 47, 1000);

        // scorehigh1
        // bot::toggleMatchLoader();
        pid::mv2pt(-21.7014, 47, 800, {.forwards = false});
        intk::scoreHigh(100);
        pid::mv2pt(-21.7014, 47, 1400, {.forwards = false});

        // ballstacks_pickup
        bot::setMatchLoader(false);
        pid::turn2hd(170, 600);
        intk::intake(100);
        pid::mv2pt(-27, 19.6489, 1000, {.maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 5});
        bot::setMatchLoader(true);
        pid::mv2pt(-26.2776, -31.0652, 1500, {.earlyExitRange = 40});
        bot::setMatchLoader(false);
        pid::mv2pt(-26.2776, -31.0652, 1500, {.earlyExitRange = 35});
        pid::mv2pt(-26.2776, -31.0652, 1000, {.maxSpeed = 50, .earlyExitRange = 10});
        bot::setMatchLoader(true);
        pid::mv2pt(-26.2776, -31.0652, 500, {.maxSpeed = 50});

        // middlegoal
        pid::turn2pt(-39.747, -13.7, 300, {.forwards = false});
        intk::setPowers(-80, 20, 0, 0);
        pid::mv2pt(-39.747, -13.7, 500, {.forwards = false});
        intk::scoreMid(100);
        pid::mv2pt(-39.747, -13.7, 1200, {.forwards = false});
        bot::setMatchLoader(false);
        intk::scoreMid(0);

        // matchload2
        pid::mv2pt(-1, -52.5, 1200);
        intk::intake(0);
        pid::turn2hd(90, 300);
        bot::setMatchLoader(true);
        pid::turn2hd(90, 250);
        intk::intake(100);
        pid::mv2pt(16.5, -52.5, 1000);

        // scorehigh2
        pid::mv2pt(-19.9499, -54, 850, {.forwards = false});
        intk::scoreHigh(100);
        pid::mv2pt(-19.9499, -54, 500, {.forwards = false});
        bot::setMatchLoader(false);
        pid::mv2pt(-19.9499, -54, 1500, {.forwards = false});
    }
    void leftQuals() {
        pid::mv2pt(0.0476588, 13.6139, 1000, {.maxSpeed = 60, .earlyExitRange = 6});
        intk::intake(100);
        pid::turn2pt(-14.7303, 27.8797, 800, {.maxSpeed = 60, .earlyExitRange = 30});
        pid::mv2pt(-15.2, 27.8797, 1000, {.maxSpeed = 80, .earlyExitRange = 10});
        bot::setMatchLoader(true);
        pid::mv2pt(-15.2, 27.8797, 500);

        pid::mv2pt(-3.50458, 33.2605, 1000, {.forwards = false, .earlyExitRange = 10});
        intk::outtake(10);
        pid::mv2pt(2.1268, 39.9448, 1000, {.forwards = false, .earlyExitRange = 4});
        intk::scoreMid(100);
        pid::mv2pt(2.1268, 39.9448, 1000, {.forwards = false});
        intk::scoreMid(0);
        bot::setMatchLoader(false);

        pid::mv2pt(-38, -1.03778, 1300);
        pid::turn2hd(180, 500);
        bot::setMatchLoader(true);
        pros::delay(250);
        intk::intake(100);
        pid::mv2pt(-38, -16, 1000, {.straight = true});

        pid::mv2pt(-38.618, 19.0416, 1000, {.forwards = false});
        intk::scoreHigh(100);
        pros::delay(1200);
        intk::stop();

        utils::turnAndSetupDescoreBackward();
        pid::mv2pt(12.6945, -18.088, 1000, {.forwards = false, .earlyExitRange = 5});
        pid::turn2hd(-20, 5000);
    }
    void rightQuals() {
        pid::mv2pt(-0.0476588, 13.6139, 1000, {.maxSpeed = 60, .earlyExitRange = 6});
        intk::intake(100);
        pid::turn2pt(14.7303, 27.8797, 800, {.maxSpeed = 60, .earlyExitRange = 30});
        pid::mv2pt(15.2, 27.8797, 1000, {.maxSpeed = 60, .earlyExitRange = 12});
        bot::setMatchLoader(true);
        pid::mv2pt(15.2, 27.8797, 500);
        bot::setMatchLoader(false);
        pid::driveWait(-0.3, -0.3, 300);

        pid::turn2pt(-0.232422, 36.5455, 800);
        pid::mv2pt(-2.732422, 39.0455, 700);
        bot::setMatchLoader(true);
        intk::outtake(100);
        pid::driveWait(0.25, 0.25, 1400);
        bot::setMatchLoader(false);
        intk::stop();

        pid::mv2pt(36.5171, 1.50574, 1250, {.forwards = false});
        pid::turn2hd(-180, 1000);
        bot::setMatchLoader(true);
        pros::delay(250);
        intk::intake(100);
        pid::mv2pt(37.5171, -15.5, 1000, {.straight = true});

        pid::mv2pt(37.7, 20.0416, 1000, {.forwards = false});
        intk::scoreHigh(100);
        pros::delay(1200);
        intk::stop();

        utils::turnAndSetupDescoreBackward();
        pid::mv2pt(12.6945, -18.088, 1000, {.forwards = false, .earlyExitRange = 5});
        pid::turn2hd(-20, 5000);
    }
} // namespace auton