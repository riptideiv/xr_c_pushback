#pragma once

#include "main.h"
namespace pid {
    struct PIDConsts {
        double kP, kI, kD;
        // maxAccel: maximum change in controller output (units per second)
        double maxAccel;
        PIDConsts(double p = 0, double i = 0, double d = 0, double maxAccel = 1000.0) : kP(p), kI(i), kD(d), maxAccel(maxAccel) {}
    };
    extern PIDConsts linearConsts, angularConsts;

    struct MoveToPointParams {
        bool forwards = true;      // Whether to move forwards or backwards
        bool straight = false;     // If true, only move straight (no turning) toward the point
        double maxSpeed = 127;     // Maximum speed (0-127)
        double minSpeed = 0;       // Minimum speed (0-127)
        double earlyExitRange = 0; // Exit early if within this distance (inches)

        double allowReverseRange = 5.0;       // range within which reversing is allowed (inches). this is to prevent overshooting the target and then turning 180 degrees
        double reverseSwitchThreshold = 15.0; // angle threshold to switch to reverse mode (degrees)
    };

    struct TurnToHeadingParams {
        bool forwards = true;      // If false, face 180 degrees away from `heading`
        double maxSpeed = 127;     // Maximum speed (0-127)
        double minSpeed = 0;       // Minimum speed (0-127)
        double earlyExitRange = 0; // Exit early if within this angle (degrees)
    };

    // drive at speeds until certain distance
    void driveDist(double lMult, double rMult, double target, bool reset = true, int timeoutMs = 2000);

    // drive at speeds for certain time
    void driveWait(double lMult, double rMult, int timeMs);

    // drive at speeds until certain angle
    void driveAngle(double lMult, double rMult, double targetAngle, bool reset = false, int timeoutMs = 2000);

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
    void mv2pt(double x, double y, int timeout, MoveToPointParams params = {}, bool async = false);

    /**
     * Turn to face a target heading (similar to lemlib turnToHeading).
     * @param heading Target heading in degrees
     * @param timeout Maximum time in milliseconds
     * @param params Turn parameters
     * @param async Whether to run asynchronously
     */
    void turn2hd(double heading, int timeout, TurnToHeadingParams params = {}, bool async = false);

    /**
     * Turn to face a target point (similar to lemlib turnToPoint).
     * @param x Target X position in inches
     * @param y Target Y position in inches
     * @param timeout Maximum time in milliseconds
     * @param params Turn parameters
     * @param async Whether to run asynchronously
     */
    void turn2pt(double x, double y, int timeout, TurnToHeadingParams params = {}, bool async = false);

    /**
     * Swing turn to face a target heading while locking one side of the drivetrain.
     * @param heading Target heading in degrees
     * @param timeout Maximum time in milliseconds
     * @param lockLeft If true, left side is locked and right side moves. If false, right side is locked and left side moves.
     * @param params Turn parameters
     * @param async Whether to run asynchronously
     */
    void swing2hd(double heading, int timeout, bool lockLeft = true, TurnToHeadingParams params = {}, bool async = false);

    /**
     * Swing turn to face a target point while locking one side of the drivetrain.
     * @param x Target X position in inches
     * @param y Target Y position in inches
     * @param timeout Maximum time in milliseconds
     * @param lockLeft If true, left side is locked and right side moves. If false, right side is locked and left side moves.
     * @param params Turn parameters
     * @param async Whether to run asynchronously
     */
    void swing2pt(double x, double y, int timeout, bool lockLeft = true, TurnToHeadingParams params = {}, bool async = false);

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