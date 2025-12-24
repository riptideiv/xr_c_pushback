#pragma once

#include "main.h"
#include "pros/rotation.hpp"
namespace chass {
    extern pros::MotorGroup mleft, mright;
    extern const double gearRatio;
    void initialize();
    void drive(double l, double r);
    void drive127(double l, double r);
    void arcade(double y, double x);
    double getLeftPos();
    double getRightPos();
    double getAvgPos();
} // namespace chass