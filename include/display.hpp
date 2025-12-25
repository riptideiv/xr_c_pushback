#pragma once
#include "main.h"

namespace display {
    extern pros::Task *displayTask;

    extern lv_obj_t *leftMotorsDisplay[3];
    extern lv_obj_t *rightMotorsDisplay[3];
    extern lv_obj_t *leftMotorsTempSquares[3];
    extern lv_obj_t *rightMotorsTempSquares[3];

    extern lv_obj_t *extraDisplay1;
    extern lv_obj_t *extraDisplay2;

    void displayMotorCurrentAndTemperature();

    void update();

    void initialize();
} // namespace display