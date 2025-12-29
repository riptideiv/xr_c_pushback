#pragma once

#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"

namespace bot {
    extern pros::adi::DigitalOut topDescore, middleDescore, upperRamp, matchLoader;

    extern bool topDescoreState, middleDescoreState, upperRampState, matchLoaderState;

    void toggleTopDescore();
    void toggleMiddleDescore();
    void toggleUpperRamp();
    void toggleMatchLoader();

    void setTopDescore(bool state);
    void setMiddleDescore(bool state);
    void setUpperRamp(bool state);
    void setMatchLoader(bool state);
} // namespace bot