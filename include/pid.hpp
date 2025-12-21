#pragma once

#include "main.h"
namespace pid {
    struct PIDConsts {
        double kP, kI, kD;
        // maxAccel: maximum change in controller output (units per second)
        double maxAccel;
        PIDConsts(double p = 0, double i = 0, double d = 0, double maxA = 1000.0) : kP(p), kI(i), kD(d), maxAccel(maxA) {}
    };
    extern PIDConsts linearConsts, angularConsts;

    /**
     * Parameters for moveToPoint function (similar to lemlib).
     */
    struct MoveToPointParams {
        bool forwards = true;      // Whether to move forwards or backwards
        double maxSpeed = 127;     // Maximum speed (0-127)
        double minSpeed = 0;       // Minimum speed (0-127)
        double earlyExitRange = 0; // Exit early if within this distance (inches)
    };

    /**
     * Parameters for turnToHeading function (similar to lemlib).
     */
    struct TurnToHeadingParams {
        double maxSpeed = 127;     // Maximum speed (0-127)
        double minSpeed = 0;       // Minimum speed (0-127)
        double earlyExitRange = 0; // Exit early if within this angle (degrees)
    };

    /**
     * Parameters for moveToPose function (similar to lemlib).
     */
    struct MoveToPoseParams {
        bool forwards = true;      // Whether to move forwards or backwards
        double maxSpeed = 127;     // Maximum speed (0-127)
        double minSpeed = 0;       // Minimum speed (0-127)
        double earlyExitRange = 0; // Exit early if within this distance (inches)
        double lead = 0.6;         // How much the chassis should curve (0-1)
    };

    /**
     * Drive straight for a specified distance.
     * @param target Distance to travel in inches
     * @param timeLimit Maximum time in milliseconds
     * @param resetPos Whether to reset encoder positions
     */
    void driveTo(double target, int timeLimit = 2000, bool resetPos = true);

    /**
     * Turn to a specified angle relative to the starting position.
     * @param target Target angle in degrees
     * @param timeLimit Maximum time in milliseconds
     * @param resetAng Whether to reset IMU heading
     */
    void turnTo(double target, int timeLimit = 2000, bool resetAng = false);

    /**
     * Move to a target point using odometry (similar to lemlib moveToPoint).
     * @param x Target X position in inches
     * @param y Target Y position in inches
     * @param timeout Maximum time in milliseconds
     * @param params Movement parameters
     * @param async Whether to run asynchronously
     */
    void moveToPoint(double x, double y, int timeout, MoveToPointParams params = {}, bool async = false);

    /**
     * Turn to face a target heading (similar to lemlib turnToHeading).
     * @param heading Target heading in degrees
     * @param timeout Maximum time in milliseconds
     * @param params Turn parameters
     * @param async Whether to run asynchronously
     */
    void turnToHeading(double heading, int timeout, TurnToHeadingParams params = {}, bool async = false);

    /**
     * Turn to face a target point (similar to lemlib turnToPoint).
     * @param x Target X position in inches
     * @param y Target Y position in inches
     * @param timeout Maximum time in milliseconds
     * @param params Turn parameters
     * @param async Whether to run asynchronously
     */
    void turnToPoint(double x, double y, int timeout, TurnToHeadingParams params = {}, bool async = false);

    /**
     * Move to a target pose (position and heading) using a boomerang controller (similar to lemlib moveToPose).
     * @param x Target X position in inches
     * @param y Target Y position in inches
     * @param theta Target heading in degrees
     * @param timeout Maximum time in milliseconds
     * @param params Movement parameters
     * @param async Whether to run asynchronously
     */
    void moveToPose(double x, double y, double theta, int timeout, MoveToPoseParams params = {}, bool async = false);

    /**
     * Follow a pure pursuit path.
     * @param path Vector of (x, y) waypoints
     * @param lookahead Lookahead distance in inches
     * @param timeout Maximum time in milliseconds
     * @param maxSpeed Maximum speed (0-127)
     * @param async Whether to run asynchronously
     */
    void followPath(const std::vector<std::pair<double, double>>& path, double lookahead, int timeout, double maxSpeed = 127, bool async = false);

    /**
     * Cancel any running async movement.
     */
    void cancelMotion();

    /**
     * Check if a movement is currently running.
     * @return true if a movement is running
     */
    bool isMoving();
} // namespace pid