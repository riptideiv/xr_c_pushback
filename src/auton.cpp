#include "auton.hpp"
#include "chassis.hpp"
#include "intake.hpp"
#include "odom.hpp"
#include "pid.hpp"
#include "pneumatics.hpp"
#include "robot.hpp"

namespace auton {
    pros::Task *autonSelectTask = nullptr;
    bool auton_running = false;
    bool auton_ran = false;

    Color selectedColor = Color::Red;
    int selectedRoute = 0;

    // Route display names - customize these for your autonomous routines
    std::vector<std::string> routeDisplay = {
        "None",
        "RightPreload",
        "LeftPreload",
        "Skills"};

    void displaySelectedAuton() {
        bot::master.print(0, 0, "%s            ", selectedColor == Color::Red ? "Red" : "Blue");
        pros::delay(50);
        bot::master.print(1, 0, "Route: %s    ", routeDisplay[selectedRoute].c_str());
    }

    void updateSelectedAuton() {
        displaySelectedAuton();
    }

    void runSelectedAuton() {
        auton_running = true;
        auton_ran = true;

        if (selectedColor == Color::Red) {
            switch (selectedRoute) {
            case 0: // None
                break;
            case 1: // RightPreload
                break;
            case 2: // LeftPreload
                break;
            case 3: // Skills
                // Add skills routine here
                break;
            }
        } else { // Blue
            switch (selectedRoute) {
            case 0: // None
                break;
            case 1: // RightPreload (mirrored)
                break;
            case 2: // LeftPreload (mirrored)
                break;
            case 3: // Skills
                // Add skills routine here
                break;
            }
        }
        auton_running = false;
        chass::drive127(0, 0);
    }

    void autonSelectLoop() {
        printf("Auton select loop started\n");
        pros::delay(500);
        bot::master.clear();
        pros::delay(100);
        displaySelectedAuton();

        while (!pros::competition::is_disabled()) {
            if (auton_running) {
                pros::delay(20);
                continue;
            }

            bool update = false;

            // Y button: Toggle color (Red/Blue)
            if (bot::master.get_digital_new_press(DIGITAL_Y)) {
                selectedColor = (selectedColor == Color::Red) ? Color::Blue : Color::Red;
                if (selectedColor == Color::Red) {
                    intk::colorSortRed = true;
                } else {
                    intk::colorSortRed = false;
                }
                update = true;
            }

            // X button: Next route
            if (bot::master.get_digital_new_press(DIGITAL_X)) {
                selectedRoute++;
                if (selectedRoute >= (int)routeDisplay.size())
                    selectedRoute = 0;
                update = true;
            }

            // B button: Previous route
            if (bot::master.get_digital_new_press(DIGITAL_B)) {
                selectedRoute--;
                if (selectedRoute < 0)
                    selectedRoute = routeDisplay.size() - 1;
                update = true;
            }

            // D-pad: Run selected auton (for testing)
            if (bot::master.get_digital_new_press(DIGITAL_LEFT) ||
                bot::master.get_digital_new_press(DIGITAL_RIGHT) ||
                bot::master.get_digital_new_press(DIGITAL_UP) ||
                bot::master.get_digital_new_press(DIGITAL_DOWN)) {
                runSelectedAuton();
                auton_ran = false;
            }

            // Bumpers: Exit selector and mark as ready
            if (bot::master.get_digital_new_press(DIGITAL_L1) ||
                bot::master.get_digital_new_press(DIGITAL_L2) ||
                bot::master.get_digital_new_press(DIGITAL_R1) ||
                bot::master.get_digital_new_press(DIGITAL_R2)) {
                auton_ran = true;
                break;
            }

            pros::delay(20); // Prevent CPU overuse

            if (!update)
                continue;

            updateSelectedAuton();
        }
        printf("Auton select loop ended.\n");
    }

    void initialize() {
        autonSelectTask = new pros::Task(autonSelectLoop);
    }
} // namespace auton
