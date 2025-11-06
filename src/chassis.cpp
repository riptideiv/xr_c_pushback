#include "chassis.hpp"
#include "main.h"
#include "pros/motors.h"

namespace chass {
    pros::MotorGroup mleft({-7, -13, -21}), mright({20, 19, 17});
    double getLeftPos() { // left motors velo in rpm
        double ans = 0;
        for (double i : mleft.get_position_all()) {
            ans += i;
        }
        return ans / 3.0;
    }
    double getRightPos() { // right motors velo in rpm
        double ans = 0;
        for (double i : mright.get_position_all()) {
            ans += i;
        }
        return ans / 3.0;
    }
    void initialize() {
        mleft.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        mright.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        mleft.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
        mright.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    }
    void drive127(double l, double r) {
        mleft.move(l);
        mright.move(r);
    }
    void drive(double l, double r) {
        drive127(l * 1.27, r * 1.27);
    }
    const struct curveConsts {
        double deadband, minOutput, base;
        double k, c; // output = k * b^x + c
        curveConsts(double dband, double minout, double b) : deadband(dband), minOutput(minout), base(b) {
            k = (127.0 - minout) / (pow(b, 127) - pow(b, dband));
            c = 127 - k * pow(b, 127);
        }
    } steeringCurve(10, 15, 1.01), throttleCurve(10, 15, 1.005);
    double applyCurve(double x, const curveConsts &curve) {
        if (fabs(x) < curve.deadband)
            return 0;
        if (x > 0)
            return curve.k * pow(curve.base, x) + curve.c;
        else // mirror curve over y axis
            return -(curve.k * pow(curve.base, -x) + curve.c);
    }
    void arcade(double y, double x) {
        int X = applyCurve(x, steeringCurve);
        int Y = applyCurve(y, throttleCurve);
        // std::cout << x << "->" << X << ' ' << y << "->" << Y << std::endl;
        drive127(Y + X, Y - X);
    }
} // namespace chass