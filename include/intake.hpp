#pragma once

#include "main.h"
#include "pros/optical.hpp"

namespace intk {
    extern pros::Motor *mwide, *mtunnel, *mtop;
    extern pros::Optical colorSortSensor;

    extern bool colorSortRed;
    extern bool doColorSort;
    extern bool doAntiStuck;
    extern bool scoringMid;

    void initialize();
    bool wrongColorDetected();
    void loop();
    void setPowers(int topPwr, int bottomPwr, bool unloadingIntake, bool scoringMidGoal);
    void intake(int power);
    void outtake(int power);
    void scoreMid(int power);
    void scoreHigh(int power);
    void stop();
} // namespace intk