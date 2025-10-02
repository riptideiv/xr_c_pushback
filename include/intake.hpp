#pragma once

#include "main.h"

namespace intk {
    extern pros::Motor *mwide, *mtunnel, *mthin;

    void initialize();
    void set(int power);
    void loop();
} // namespace intk