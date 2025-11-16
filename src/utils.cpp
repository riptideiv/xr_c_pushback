#include "utils.hpp"
#include "chassis.hpp"
#include "pid.hpp"
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
} // namespace utils