#pragma once

#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"

namespace bot {
    extern pros::adi::AnalogOut descoreArm, matchLoader;
    void toggleDescoreArm();
    void toggleMatchLoader();
} // namespace bot