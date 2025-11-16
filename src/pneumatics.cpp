#include "pneumatics.hpp"
#include "pros/adi.hpp"

namespace bot {
    pros::adi::DigitalOut descoreArm('A');
    pros::adi::DigitalOut matchLoader('B');
    void toggleDescoreArm() {
        static bool descoreArmState = false;
        descoreArmState = !descoreArmState;
        descoreArm.set_value(descoreArmState);
    }
    void toggleMatchLoader() {
        static bool matchLoaderState = false;
        matchLoaderState = !matchLoaderState;
        matchLoader.set_value(matchLoaderState);
    }
} // namespace bot