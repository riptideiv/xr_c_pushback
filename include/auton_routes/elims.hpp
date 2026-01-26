#pragma once
#include "chassis.hpp"
#include "intake.hpp"
#include "odom.hpp"
#include "pid.hpp"
#include "pneumatics.hpp"
#include "utils.hpp"
namespace auton {
    void leftElims() {
        bot::setTopDescore(true);
        pid::mv2pt(0.0476588, 13.6139, 1000, {.maxSpeed = 60, .earlyExitRange = 6});
        intk::intake(100);
        pid::turn2pt(-14.7303, 27.8797, 800, {.maxSpeed = 60, .earlyExitRange = 30});
        pid::mv2pt(-15.2, 27.8797, 1000, {.maxSpeed = 80, .earlyExitRange = 10});
        bot::setMatchLoader(true);
        pid::mv2pt(-15.2, 27.8797, 500);
        pid::driveDist(0.2, 0.2, -1);

        pid::mv2pt(-38, -1.03778, 600);
        bot::setMatchLoader(false);
        pid::mv2pt(-38, -1.03778, 700);
        pid::turn2hd(180, 500);
        bot::setMatchLoader(true);
        pros::delay(250);
        intk::intake(100);
        pid::mv2pt(-38, -16, 1000, {.straight = true});

        pid::mv2pt(-38.818, 19.0416, 1000, {.forwards = false});
        intk::scoreHigh(100);
        pros::delay(2000);
        intk::stop();

        utils::turnAndSetupDescoreBackward();
        pid::mv2pt(12.6945, -18.088, 1000, {.forwards = false, .earlyExitRange = 5});
        pid::turn2hd(-20, 5000);
    }
    void rightElims() {
        bot::setTopDescore(true);
        pid::mv2pt(-0.0476588, 13.6139, 1000, {.maxSpeed = 60, .earlyExitRange = 6});
        intk::intake(100);
        pid::turn2pt(14.7303, 27.8797, 800, {.maxSpeed = 60, .earlyExitRange = 30});
        pid::mv2pt(15.2, 27.8797, 1000, {.maxSpeed = 60, .earlyExitRange = 12});
        bot::setMatchLoader(true);
        pid::mv2pt(15.2, 27.8797, 500);
        pid::driveDist(0.2, 0.2, -1);

        pid::mv2pt(37.0171, 1.50574, 600);
        bot::setMatchLoader(false);
        pid::mv2pt(37.0171, 1.50574, 700);
        pid::turn2hd(-180, 350);
        bot::setMatchLoader(true);
        pros::delay(250);
        intk::intake(100);
        pid::mv2pt(37.5171, -15.5, 1000, {.straight = true});

        pid::mv2pt(37.7, 20.0416, 1000, {.forwards = false});
        intk::scoreHigh(100);
        pros::delay(2000);
        intk::stop();

        utils::turnAndSetupDescoreBackward();
        pid::mv2pt(12.6945, -18.088, 1000, {.forwards = false, .earlyExitRange = 5});
        pid::turn2hd(-20, 5000);
        ;
    }
} // namespace auton