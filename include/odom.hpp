#pragma once

#include "main.h"

namespace odom {
    extern double xpos, ypos, xvelo, yvelo, speed, angvelo;
    extern pros::Task *odomTask;
    void odomLoop();
    void initialize();
    void pdrive(int targ, int t);

    double getVertPos();  // vertical tracking wheel position in inches
    double getHorizPos(); // horizontal tracking wheel position in inches

    /**
     * Get the current X position in inches.
     */
    double getX();

    /**
     * Get the current Y position in inches.
     */
    double getY();

    /**
     * Get the current heading in degrees.
     */
    double getHeading();

    /**
     * Set the current pose (position and heading).
     * @param x X position in inches
     * @param y Y position in inches
     * @param heading Heading in degrees
     */
    void setPose(double x, double y, double heading);

    /**
     * Reset the pose to (0, 0, 0).
     */
    void resetPose();

    /**
     * Calculate the distance from the current position to a target point.
     * @param targetX Target X position in inches
     * @param targetY Target Y position in inches
     * @return Distance in inches
     */
    double distanceTo(double targetX, double targetY);

    /**
     * Calculate the angle from the current position to a target point.
     * @param targetX Target X position in inches
     * @param targetY Target Y position in inches
     * @return Angle in degrees
     */
    double angleTo(double targetX, double targetY);

    /**
     * Print the current odometry readings for debugging.
     */
    void debugPrint();
} // namespace odom