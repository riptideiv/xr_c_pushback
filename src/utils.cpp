#include "utils.hpp"
#include "chassis.hpp"
#include "odom.hpp"
#include "pid.hpp"
#include "pneumatics.hpp"
#include "robot.hpp"
#include "sensors.hpp"
#include <iostream>

namespace utils {
    int sign(double val) {
        if (val > 0)
            return 1;
        if (val < 0)
            return -1;
        return 0;
    }

    void turnAndSetupDescoreBackward() {
        bot::setMatchLoader(false);
        odom::setPose(0, 0, 0);
        pid::driveAngle(1, -1, 30, false, 500);
        pid::turn2hd(50, 250);
        bot::setTopDescore(false);
        pid::swing2hd(0, 500, true, {.maxSpeed = 70});
    }

    void turnAndDescore() {
        bot::setMatchLoader(false);
        odom::setPose(0, 0, 0);
        bot::setTopDescore(true);
        pid::driveAngle(-1, 1, -110, false, 850);
        pid::turn2hd(-170, 420);
        odom::setPose(0, 0, 10);
        pid::mv2pt(-1, -10, 700, {.forwards = false, .straight = true, .earlyExitRange = 4});
        bot::setTopDescore(false);
        pid::mv2pt(-1, -10, 300, {.forwards = false, .straight = true, .earlyExitRange = 2});
        pid::turn2hd(0, 300, {.minSpeed = 15, .earlyExitRange = 5});
        pid::mv2pt(0, 2, 1000, {.straight = true, .maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 5});
        chass::drive(0, 0);
    }

    // ============== PID Tuning ==============

    void runFwdBwdTest(double target, int timeout) {
        pid::driveTo(target, timeout);
        double pos = chass::getAvgPos();
        std::cout << "forward: " << pos << std::endl;

        pid::driveTo(-target, timeout);
        pos = chass::getAvgPos();
        std::cout << "backward: " << pos << std::endl;
    }

    void runTurnTest(double target, int timeout) {
        double startAngle = bot::imu.get_rotation();
        pid::turnTo(target, timeout);
        double endAngle = bot::imu.get_rotation();
        std::cout << "turn: " << (endAngle - startAngle) << std::endl;

        startAngle = bot::imu.get_rotation();
        pid::turnTo(0, timeout);
        endAngle = bot::imu.get_rotation();
        std::cout << "turn back: " << (endAngle - startAngle) << std::endl;
    }

    void runAngularPID_kPs(double kD, double begin, double end, double step, double target, int timeout) {
        pid::angularConsts.kD = kD;
        for (double kP = begin; kP <= end; kP += step) {
            std::cout << "kP: " << kP << std::endl;
            pid::angularConsts.kP = kP;
            // Use the turn test helper to evaluate this kP
            runTurnTest(target, timeout);
        }
    }

    void runAngularPID_kDs(double kP, double begin, double end, double step, double target, int timeout) {
        pid::angularConsts.kP = kP;
        for (double kD = begin; kD <= end; kD += step) {
            std::cout << "kD: " << kD << std::endl;
            pid::angularConsts.kD = kD;
            // Use the turn test helper to evaluate this kD
            runTurnTest(target, timeout);
        }
    }

    void runLateralPID_kPs(double kD, double begin, double end, double step, double target, int timeout) {
        pid::linearConsts.kD = kD;
        for (double kP = begin; kP <= end; kP += step) {
            std::cout << "kP: " << kP << std::endl;
            pid::linearConsts.kP = kP;
            // Use the forward/backward test helper to evaluate this kP
            runFwdBwdTest(target, timeout);
        }
    }

    void runLateralPID_kDs(double kP, double begin, double end, double step, double target, int timeout) {
        pid::linearConsts.kP = kP;
        for (double kD = begin; kD <= end; kD += step) {
            std::cout << "kD: " << kD << std::endl;
            pid::linearConsts.kD = kD;
            // Use the forward/backward test helper to evaluate this kD
            runFwdBwdTest(target, timeout);
        }
    }

    void runLateralBSearchkP(double kD, double l, double r, double target, int timeout) {
        pid::linearConsts.kD = kD;
        while (true) {
            double m = (l + r) / 2;
            pid::linearConsts.kP = m;
            std::cout << l << ", " << r << ", mid: " << m << std::endl;
            runFwdBwdTest(target, timeout);
            while (true) {
                pros::delay(20);
                if (bot::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
                    l = m;
                    break;
                }
                if (bot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                    r = m;
                    break;
                }
                if (bot::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                    break;
                }
            }
        }
    }

    void runLateralBSearchkD(double kP, double l, double r, double target, int timeout) {
        pid::linearConsts.kP = kP;
        while (true) {
            double m = (l + r) / 2;
            pid::linearConsts.kD = m;
            std::cout << l << ", " << r << ", mid: " << m << std::endl;
            runFwdBwdTest(target, timeout);
            while (true) {
                pros::delay(20);
                if (bot::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
                    l = m;
                    break;
                }
                if (bot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                    r = m;
                    break;
                }
                if (bot::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                    break;
                }
            }
        }
    }

    void runAngularBSearchkP(double kD, double l, double r, double target, int timeout) {
        pid::angularConsts.kD = kD;
        while (true) {
            double m = (l + r) / 2;
            pid::angularConsts.kP = m;
            std::cout << l << ", " << r << ", mid: " << m << std::endl;
            runTurnTest(target, timeout);
            while (true) {
                pros::delay(20);
                if (bot::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
                    l = m;
                    break;
                }
                if (bot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                    r = m;
                    break;
                }
                if (bot::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                    break;
                }
            }
        }
    }

    void runAngularBSearchkD(double kP, double l, double r, double target, int timeout) {
        pid::angularConsts.kP = kP;
        while (true) {
            double m = (l + r) / 2;
            pid::angularConsts.kD = m;
            std::cout << l << ", " << r << ", mid: " << m << std::endl;
            runTurnTest(target, timeout);
            while (true) {
                pros::delay(20);
                if (bot::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
                    l = m;
                    break;
                }
                if (bot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                    r = m;
                    break;
                }
                if (bot::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                    break;
                }
            }
        }
    }

    void findTrackingRadius() {
        double avgHoriz = 0;
        double avgVert = 0;
        double avgL = 0;
        double avrR = 0;
        for (int i = 0; i < 5; i++) {
            int t = clock();
            int speed = 30 + (i * 10);
            std::cout << "Run " << (i + 1) << " with speed: " << speed << '\n';
            chass::drive(speed, -speed);
            pros::delay(1000);
            bot::horizEnc.reset();
            double initL = chass::getLeftPos();
            double initR = chass::getRightPos();
            double initAngle = bot::imu.get_rotation();
            pros::delay(3000);
            double circumf = M_PI * 2.75; // 2.75 inch diameter tracking wheels
            double posHoriz = bot::horizEnc.get_position() / 36000.0 * circumf;
            double posVert = bot::vertEnc.get_position() / 36000.0 * circumf;
            double posL = chass::getLeftPos() - initL;
            double posR = chass::getRightPos() - initR;
            double angle = (bot::imu.get_rotation() - initAngle) / 180.0 * M_PI;
            std::cout << "horizRadius: " << (posHoriz / angle) << '\n'
                      << "vertRadius: " << (posVert / angle) << '\n'
                      << "leftRadius: " << (posL / angle) << '\n'
                      << "rightRadius: " << (posR / angle) << '\n';
            avgHoriz += (posHoriz / angle);
            avgVert += (posVert / angle);
            avgL += (posL / angle);
            avrR += (posR / angle);
        }
        std::cout << "average horiz radius: " << (avgHoriz / 5) << '\n'
                  << "average vert radius: " << (avgVert / 5) << '\n'
                  << "avgL: " << (avgL / 5) << '\n'
                  << "avgR: " << (avrR / 5) << '\n';
    }
} // namespace utils