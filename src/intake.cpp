#include "intake.hpp"
#include "pneumatics.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "utils.hpp"

namespace intk {
    pros::Motor *mtop = new pros::Motor(-7),
                *mbottom = new pros::Motor(6);
    pros::Task *tloop = nullptr;

    pros::Optical colorSortSensor(12);

    bool doColorSort;
    bool colorSortRed;
    bool doAntiStuck;

    bool unloading;
    bool scoringMid;

    struct intakeRoller {
        pros::Motor *motor;
        int speed, speedLimit = 1000;
        bool stuck() {
            return abs(speed) > 0 && motor->get_actual_velocity() / speed < 0.1;
        }
        void spin(int speedPct) {
            int spd = speedPct;
            if (abs(spd) > speedLimit)
                spd = utils::sign(spd) * speedLimit;
            motor->move_voltage(120 * spd);
        }
        void spin() {
            int spd = speed;
            if (abs(spd) > speedLimit)
                spd = utils::sign(spd) * speedLimit;
            motor->move_voltage(120 * spd);
        }
        void limSpeed(int spdLimit) {
            speedLimit = spdLimit;
        }
    } bottom = {mbottom, 0}, top = {mtop, 0};
    int power = 0;

    bool throwAway;

    int stuckFor;

    int revTime;
    int topRevTime;

    int prevPwr;
    int startUpTime;

    const int loopDelay = 3;

    void initialize() {
        mbottom->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        mtop->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

        doColorSort = false;
        colorSortRed = true;
        doAntiStuck = true;

        unloading = false;
        scoringMid = false;

        throwAway = false;

        stuckFor = 0;

        revTime = 0;
        topRevTime = 0;

        prevPwr = 0;
        startUpTime = 0;

        tloop = new pros::Task([]() {while(1) loop(); });
    }

    void intake(int pwr) {
        top.speed = 0;
        bottom.speed = pwr;
        power = pwr;
        unloading = false;
        scoringMid = false;
    }

    void outtake(int pwr) {
        top.speed = -pwr;
        bottom.speed = -pwr;
        power = pwr;
        unloading = true;
        scoringMid = false;
    }

    void scoreMid(int pwr) {
        top.speed = 0;
        bottom.speed = pwr;
        power = pwr;
        unloading = true;
        scoringMid = true;
    }

    void scoreHigh(int pwr) {
        top.speed = pwr;
        bottom.speed = pwr;
        power = pwr;
        unloading = true;
        scoringMid = false;
    }

    void stop() {
        bottom.speed = top.speed = power = 0;
    }

    void colorSort() {
        if ((colorSortSensor.get_hue() < 30 || colorSortSensor.get_hue() > 340) && !colorSortRed ||
            (colorSortSensor.get_hue() > 120 && colorSortSensor.get_hue() < 270) && colorSortRed) {
            topRevTime = 300;
        }
    }

    void antiStuck() {
        if (revTime <= 0 && (top.stuck() || bottom.stuck())) {
            stuckFor += loopDelay;
        } else {
            stuckFor = 0;
        }

        if (stuckFor > 50) {
            revTime = 100;
            stuckFor = 0;
        }
    }

    void loop() {
        // // debug
        // topRevTime = 1000;

        pros::delay(loopDelay);
        if (startUpTime <= 0) {
            if (doColorSort) {
                colorSort();
            }
            if (doAntiStuck) {
                antiStuck();
            }
        } else {
            startUpTime -= loopDelay;
        }

        // give startup time when intake starts
        if (prevPwr == 0 && power > 0) {
            startUpTime = 150;
        }

        // record previous power to check for changes
        prevPwr = power;

        if (topRevTime > 0) {
            revTime = 0;       // cancel bottom reverse if top is reversing
            startUpTime = 100; // give startup time if the top intake is auto-reversing
            topRevTime -= loopDelay;
            bottom.spin();
            top.spin(-100);
        } else if (revTime > 0) {
            startUpTime = 100; // give startup time if the intake is auto-reversing
            revTime -= loopDelay;
            bottom.spin(utils::sign(bottom.speed) * -100);
            top.spin(utils::sign(top.speed) * -100);
        } else {
            // normal operation
            bottom.spin();
            top.spin();
            if (scoringMid) { // retract upper ramp when scoring mid goal
                bot::upperRamp.set_value(true);
            } else {
                bot::upperRamp.set_value(false);
            }
            // if (unloading && top.speed != 0) {
            //     double topVelo = top.motor->get_actual_velocity();
            //     // std::cout << tunnelVelo << std::endl;
            //     if (fabs(topVelo) < 100)
            //         tunnel.limSpeed(10);
            //     else
            //         tunnel.limSpeed(1000);
            // } else {
            //     tunnel.limSpeed(1000);
            // }
        }
    }
} // namespace intk