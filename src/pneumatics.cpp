#include "pneumatics.hpp"
#include "pros/adi.hpp"

namespace bot {
    pros::adi::DigitalOut topDescore('F'), middleDescore('H'), upperRamp('G'), matchLoader('D');

    void toggleTopDescore() {
        static bool state = false;
        state = !state;
        topDescore.set_value(state);
    }

    void toggleMiddleDescore() {
        static bool state = false;
        state = !state;
        middleDescore.set_value(state);
    }

    void toggleUpperRamp() {
        static bool state = false;
        state = !state;
        upperRamp.set_value(state);
    }

    void toggleMatchLoader() {
        static bool state = false;
        state = !state;
        matchLoader.set_value(state);
    }
} // namespace bot