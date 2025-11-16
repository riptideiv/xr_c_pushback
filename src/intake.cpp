#include "intake.hpp"
#include "pros/motors.hpp"
#include "utils.hpp"

namespace intk {
    pros::Motor *mwide = new pros::Motor(-4),
                *mtunnel = new pros::Motor(5),
                *mtop = new pros::Motor(-10);
    pros::Task *tloop = nullptr;

    bool doColorSort;
    bool colorSortRed;
    bool doAntiStuck;

    bool unloading;

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
    } tunnel = {mtunnel, 0}, wide = {mwide, 0}, top = {mtop, 0};
    int power = 0;

    bool throwAway;

    int stuckFor;

    int revTime;
    int frRevTime;

    int prevPwr;
    int startUpTime;

    const int loopDelay = 3;

    void initialize() {
        mwide->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        mtunnel->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        mtop->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

        doColorSort = false;
        colorSortRed = true;
        doAntiStuck = true;

        unloading = false;

        throwAway = false;

        stuckFor = 0;

        revTime = 0;
        frRevTime = 0;

        prevPwr = 0;
        startUpTime = 0;

        tloop = new pros::Task([]() {while(1) loop(); });
    }

    void intake(int pwr) {
        tunnel.speed = pwr;
        wide.speed = pwr;
        power = pwr;
        unloading = false;
    }

    void outtake(int pwr) {
        tunnel.speed = -pwr;
        wide.speed = -pwr;
        power = pwr;
        unloading = true;
    }

    void scoreMid(int pwr) {
        tunnel.speed = -pwr;
        wide.speed = pwr;
        top.speed = -pwr;
        power = pwr;
        unloading = true;
    }

    void scoreHigh(int pwr) {
        tunnel.speed = -pwr;
        wide.speed = pwr;
        top.speed = pwr;
        power = pwr;
        unloading = true;
    }

    void stop() {
        tunnel.speed = wide.speed = top.speed = power = 0;
    }

    void colorSort() {
    }

    void antiStuck() {
        if (revTime <= 0 && (top.stuck() || tunnel.stuck() || wide.stuck())) {
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

        if (revTime > 0) {
            startUpTime = 100; // give startup time if the intake is auto-reversing
            revTime -= loopDelay;
            top.spin(utils::sign(top.speed) * -100);
            tunnel.spin(utils::sign(tunnel.speed) * -100);
            wide.spin(utils::sign(wide.speed) * -100);
        } else {
            // normal operation
            tunnel.spin();
            wide.spin();
            top.spin();
            if (unloading && top.speed != 0) {
                double topVelo = top.motor->get_actual_velocity();
                // std::cout << tunnelVelo << std::endl;
                if (fabs(topVelo) < 100)
                    tunnel.limSpeed(10);
                else
                    tunnel.limSpeed(1000);
            } else {
                tunnel.limSpeed(1000);
            }
        }
    }
} // namespace intk