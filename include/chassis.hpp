#pragma once

#include "main.h"
namespace chass {
    extern pros::MotorGroup mleft, mright;
    void initialize();
    void drive(double l, double r);
    void drive127(double l, double r);
    void arcade(double y, double x);
    double getLeftPos();
    double getRightPos();
} // namespace chass