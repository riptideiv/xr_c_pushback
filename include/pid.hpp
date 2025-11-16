#pragma once

#include "main.h"
namespace pid {
    struct PIDConsts {
        double kP, kI, kD;
        // maxAccel: maximum change in controller output (units per second)
        double maxAccel;
        PIDConsts(double p = 0, double i = 0, double d = 0, double maxA = 1000.0) : kP(p), kI(i), kD(d), maxAccel(maxA) {}
    };
    extern PIDConsts linearConsts, angularConsts;
    void driveTo(double target, int timeLimit = 2000, bool resetPos = true);
    void turnTo(double target, int timeLimit = 2000, bool resetAng = false);
} // namespace pid