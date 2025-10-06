#pragma once

#include "main.h"

namespace intk {
    extern pros::Motor *mwide, *mtunnel, *mthin;

    void initialize();
    void set(int power);
    void loop();
    void intake(int power);
    void outtake(int power);
    void scoreMid(int power);
    void scoreHigh(int power);
    void stop();
} // namespace intk