#pragma once

#include "main.h"
namespace chass {
    extern pros::MotorGroup mleft, mright;
    void initialize();
    void drive(int l, int r);
    void arcade(double y, double x);
} // namespace chass