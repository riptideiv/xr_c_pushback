#pragma once

#include "chassis.hpp"
#include "intake.hpp"
#include "pneumatics.hpp"

namespace bot {
    extern pros::Controller master;

    void initialize();

} // namespace bot