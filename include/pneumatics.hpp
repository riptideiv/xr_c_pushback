#pragma once

#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"

namespace bot {
    extern pros::adi::DigitalOut topDescore, middleDescore, upperRamp, matchLoader;
    void toggleTopDescore();
    void toggleMiddleDescore();
    void toggleUpperRamp();
    void toggleMatchLoader();
} // namespace bot