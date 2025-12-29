#include "pneumatics.hpp"
#include "pros/adi.hpp"

namespace bot {
    pros::adi::DigitalOut topDescore('F'), middleDescore('H'), upperRamp('G'), matchLoader('D');

    bool topDescoreState = false, middleDescoreState = false, upperRampState = false, matchLoaderState = false;

    void toggleTopDescore() {
        topDescore.set_value(topDescoreState = !topDescoreState);
    }

    void toggleMiddleDescore() {
        middleDescore.set_value(middleDescoreState = !middleDescoreState);
    }

    void toggleUpperRamp() {
        upperRamp.set_value(upperRampState = !upperRampState);
    }

    void toggleMatchLoader() {
        matchLoader.set_value(matchLoaderState = !matchLoaderState);
    }

    void setTopDescore(bool state) {
        topDescore.set_value(state);
        topDescoreState = state;
    }

    void setMiddleDescore(bool state) {
        middleDescore.set_value(state);
        middleDescoreState = state;
    }

    void setUpperRamp(bool state) {
        upperRamp.set_value(state);
        upperRampState = state;
    }

    void setMatchLoader(bool state) {
        matchLoader.set_value(state);
        matchLoaderState = state;
    }
} // namespace bot