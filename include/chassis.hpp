#pragma once

#include "main.h"
#include "pros/rotation.hpp"
namespace chass {
    extern pros::MotorGroup mleft, mright;
    extern pros::Rotation horizEnc, vertEnc;
    extern const double gearRatio;
    extern const double horizOffset, vertOffset; // horiz offset = rightward is +, vert offset = forward is +
    void initialize();
    void drive(double l, double r);
    void drive127(double l, double r);
    void arcade(double y, double x);
    double getLeftPos();
    double getRightPos();
    double getAvgPos();
} // namespace chass