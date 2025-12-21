#pragma once

#include "main.h"
#include <string>
#include <vector>

namespace auton {
    extern pros::Task *autonSelectTask;
    extern bool auton_running;
    extern bool auton_ran;

    enum class Color { Red, Blue };

    extern Color selectedColor;
    extern int selectedRoute;

    // Route display names
    extern std::vector<std::string> routeDisplay;

    /**
     * Initialize the auton selector - starts the selection loop task.
     */
    void initialize();

    /**
     * Display the currently selected auton on the controller screen.
     */
    void displaySelectedAuton();

    /**
     * Update settings when selected auton changes.
     */
    void updateSelectedAuton();

    /**
     * Run the currently selected autonomous routine.
     */
    void runSelectedAuton();

    /**
     * Main loop for auton selection - runs as a task.
     */
    void autonSelectLoop();
} // namespace auton
