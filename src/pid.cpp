#include "pid.hpp"
#include "chassis.hpp"
#include "odom.hpp"
#include "sensors.hpp"
#include "utils.hpp"
#include <atomic>
#include <cmath>
#include <numbers>

namespace pid {
    PIDConsts linearConsts(6, 0.0, 250, 10000000);
    PIDConsts angularConsts(6, 0.0, 375, 10000000);

    // Async motion control
    static std::atomic<bool> motionRunning{false};
    static std::atomic<bool> motionCancelled{false};
    static pros::Task *asyncTask = nullptr;

    void driveDist(double lMult, double rMult, double target, bool reset, int timeoutMs) {
        double startPos = 0;
        if (reset)
            startPos = odom::getVertPos();
        int startTime = pros::millis();
        if (target >= 0) {
            while (odom::getVertPos() - startPos < target) {
                if (timeoutMs > 0 && pros::millis() - startTime >= timeoutMs)
                    break;
                chass::drive127(lMult * 127, rMult * 127);
                pros::delay(10);
            }
        } else {
            while (odom::getVertPos() - startPos > target) {
                if (timeoutMs > 0 && pros::millis() - startTime >= timeoutMs)
                    break;
                chass::drive127(lMult * -127, rMult * -127);
                pros::delay(10);
            }
        }
    }

    void driveWait(double lMult, double rMult, int timeMs) {
        int startTime = pros::millis();
        while (pros::millis() - startTime < timeMs) {
            chass::drive127(lMult * 127, rMult * 127);
            pros::delay(10);
        }
        // chass::drive127(0, 0);
    }

    void driveAngle(double lMult, double rMult, double targetAngle, bool reset, int timeoutMs) {
        if (reset) {
            bot::imu.set_rotation(0);
        }
        double imu = bot::imu.get_rotation();
        double error = targetAngle - imu;
        double prevError = error;
        int startTime = pros::millis();
        while (utils::sign(error) == utils::sign(prevError) && error != 0) {
            if (timeoutMs > 0 && pros::millis() - startTime >= timeoutMs)
                break;
            prevError = error;
            error = targetAngle - bot::imu.get_rotation();
            chass::drive127(lMult * 127, rMult * 127);
            pros::delay(10);
        }
    } // Helper function to normalize angle to [-180, 180]
    static double normalizeAngle(double angle) {
        while (angle > 180)
            angle -= 360;
        while (angle < -180)
            angle += 360;
        return angle;
    }

    // Helper function to clamp a value
    static double clamp(double value, double min, double max) {
        if (value > max)
            return max;
        if (value < min)
            return min;
        return value;
    }

    void driveTo(double target, int timeLimit, bool resetPos) {
        if (resetPos) {
            chass::mleft.set_zero_position_all(0);
            chass::mright.set_zero_position_all(0);
        }
        // std::cout << chass::getAvgPos() << std::endl;
        double error = target - chass::getAvgPos();
        double prevError = error;
        double integral = 0;
        const int loopMs = 10;
        double prevOutput = 0.0;
        int startTime = pros::millis();
        while (pros::millis() - startTime < timeLimit) {
            pros::delay(loopMs);
            error = target - chass::getAvgPos();
            integral += error * loopMs;
            double derivative = (error - prevError) / (double)loopMs;
            double desired = linearConsts.kP * error + linearConsts.kI * integral + linearConsts.kD * derivative;
            // clamp desired
            if (desired > 127)
                desired = 127;
            if (desired < -127)
                desired = -127;
            // rate limit (max change per loop)
            double maxDelta = linearConsts.maxAccel * (loopMs / 1000.0);
            double delta = desired - prevOutput;
            if (delta > maxDelta)
                delta = maxDelta;
            if (delta < -maxDelta)
                delta = -maxDelta;
            double output = prevOutput + delta;
            chass::drive127(output, output);
            prevError = error;
            prevOutput = output;
        }
        // ramp down to zero gracefully
        {
            const int loopMs2 = 10;
            double maxDelta = linearConsts.maxAccel * (loopMs2 / 1000.0);
            while (fabs(prevOutput) > 1.0) {
                double delta = (0.0 - prevOutput);
                if (delta > maxDelta)
                    delta = maxDelta;
                if (delta < -maxDelta)
                    delta = -maxDelta;
                prevOutput += delta;
                chass::drive127(prevOutput, prevOutput);
                pros::delay(loopMs2);
            }
            chass::drive127(0, 0);
        }
    }
    void turnTo(double target, int timeLimit, bool resetAng) {
        if (resetAng) {
            bot::imu.set_heading(0);
        }
        double error = target - bot::imu.get_rotation();
        double prevError = error;
        double integral = 0;
        const int loopMs = 10;
        double prevOutput = 0.0;
        int startTime = pros::millis();
        while (pros::millis() - startTime < timeLimit) {
            pros::delay(loopMs);
            error = target - bot::imu.get_rotation();
            integral += error * loopMs;
            double derivative = (error - prevError) / (double)loopMs;
            double desired = angularConsts.kP * error + angularConsts.kI * integral + angularConsts.kD * derivative;
            if (desired > 127)
                desired = 127;
            if (desired < -127)
                desired = -127;
            double maxDelta = angularConsts.maxAccel * (loopMs / 1000.0);
            double delta = desired - prevOutput;
            if (delta > maxDelta)
                delta = maxDelta;
            if (delta < -maxDelta)
                delta = -maxDelta;
            double output = prevOutput + delta;
            chass::drive127(output, -output);
            prevError = error;
            prevOutput = output;
        }
        // ramp down to zero gracefully
        {
            const int loopMs2 = 10;
            double maxDelta = angularConsts.maxAccel * (loopMs2 / 1000.0);
            while (fabs(prevOutput) > 1.0) {
                double delta = (0.0 - prevOutput);
                if (delta > maxDelta)
                    delta = maxDelta;
                if (delta < -maxDelta)
                    delta = -maxDelta;
                prevOutput += delta;
                chass::drive127(prevOutput, -prevOutput);
                pros::delay(loopMs2);
            }
            chass::drive127(0, 0);
        }
    }

    void moveToPointSync(double x, double y, int timeout, MoveToPointParams params) {
        motionRunning = true;
        motionCancelled = false;

        double prevLinearError = odom::distanceTo(x, y);
        double linearIntegral = 0;
        double prevAngularError = 0;
        double angularIntegral = 0;
        double prevLinearOutput = 0;
        double prevAngularOutput = 0;

        const int loopMs = 10;
        int startTime = pros::millis();

        while (pros::millis() - startTime < timeout && !motionCancelled) {
            pros::delay(loopMs);

            // Calculate distance to target
            double distance = odom::distanceTo(x, y);

            // Check early exit
            if (params.earlyExitRange > 0 && distance < params.earlyExitRange) {
                break;
            }

            // Calculate angle to target
            double targetAngle = odom::angleTo(x, y);
            if (!params.forwards) {
                targetAngle = normalizeAngle(targetAngle + 180);
            }
            double currentAngle = odom::getHeading();
            double angularError = normalizeAngle(targetAngle - currentAngle);

            // Linear PID
            double linearError = distance;
            linearIntegral += linearError * loopMs;
            double linearDerivative = (linearError - prevLinearError) / (double)loopMs;
            double linearOutput = linearConsts.kP * linearError + linearConsts.kI * linearIntegral + linearConsts.kD * linearDerivative;

            // Apply direction
            if (!params.forwards) {
                linearOutput = -linearOutput;
            }

            // Angular PID
            angularIntegral += angularError * loopMs;
            double angularDerivative = (angularError - prevAngularError) / (double)loopMs;
            double angularOutput = angularConsts.kP * angularError + angularConsts.kI * angularIntegral + angularConsts.kD * angularDerivative;

            // If straight flag is set, disable angular output and reset angular internals
            if (params.straight) {
                angularOutput = 0;
                angularIntegral = 0;
                prevAngularOutput = 0;
                prevAngularError = 0;
            }

            // Clamp outputs
            linearOutput = clamp(linearOutput, -params.maxSpeed, params.maxSpeed);
            angularOutput = clamp(angularOutput, -params.maxSpeed, params.maxSpeed);

            // Apply minimum speed
            if (fabs(linearOutput) < params.minSpeed && fabs(linearOutput) > 0.1) {
                linearOutput = (linearOutput > 0) ? params.minSpeed : -params.minSpeed;
            }

            // Rate limit
            double maxLinearDelta = linearConsts.maxAccel * (loopMs / 1000.0);
            double linearDelta = clamp(linearOutput - prevLinearOutput, -maxLinearDelta, maxLinearDelta);
            linearOutput = prevLinearOutput + linearDelta;

            double maxAngularDelta = angularConsts.maxAccel * (loopMs / 1000.0);
            double angularDelta = clamp(angularOutput - prevAngularOutput, -maxAngularDelta, maxAngularDelta);
            angularOutput = prevAngularOutput + angularDelta;

            // Calculate wheel speeds
            double leftSpeed = linearOutput + angularOutput;
            double rightSpeed = linearOutput - angularOutput;

            // Normalize if exceeding max
            double maxOutput = std::max(fabs(leftSpeed), fabs(rightSpeed));
            if (maxOutput > params.maxSpeed) {
                leftSpeed = leftSpeed / maxOutput * params.maxSpeed;
                rightSpeed = rightSpeed / maxOutput * params.maxSpeed;
            }

            chass::drive127(leftSpeed, rightSpeed);

            prevLinearError = linearError;
            prevAngularError = angularError;
            prevLinearOutput = linearOutput;
            prevAngularOutput = angularOutput;
        }

        chass::drive127(0, 0);
        motionRunning = false;
    }

    void moveToPoint(double x, double y, int timeout, MoveToPointParams params, bool async) {
        if (async) {
            if (asyncTask != nullptr) {
                cancelMotion();
            }
            asyncTask = new pros::Task([x, y, timeout, params]() {
                moveToPointSync(x, y, timeout, params);
            });
        } else {
            moveToPointSync(x, y, timeout, params);
        }
    }

    void turnToHeadingSync(double heading, int timeout, TurnToHeadingParams params) {
        motionRunning = true;
        motionCancelled = false;

        double prevError = normalizeAngle(heading - odom::getHeading());
        double integral = 0;
        double prevOutput = 0;

        const int loopMs = 10;
        int startTime = pros::millis();

        while (pros::millis() - startTime < timeout && !motionCancelled) {
            pros::delay(loopMs);

            double error = normalizeAngle(heading - odom::getHeading());

            // Check early exit
            if (params.earlyExitRange > 0 && fabs(error) < params.earlyExitRange) {
                break;
            }

            // PID
            integral += error * loopMs;
            double derivative = (error - prevError) / (double)loopMs;
            double output = angularConsts.kP * error + angularConsts.kI * integral + angularConsts.kD * derivative;

            // Clamp
            output = clamp(output, -params.maxSpeed, params.maxSpeed);

            // Apply minimum speed
            if (fabs(output) < params.minSpeed && fabs(output) > 0.1) {
                output = (output > 0) ? params.minSpeed : -params.minSpeed;
            }

            // Rate limit
            double maxDelta = angularConsts.maxAccel * (loopMs / 1000.0);
            double delta = clamp(output - prevOutput, -maxDelta, maxDelta);
            output = prevOutput + delta;

            chass::drive127(output, -output);

            prevError = error;
            prevOutput = output;
        }

        chass::drive127(0, 0);
        motionRunning = false;
    }

    void turnToHeading(double heading, int timeout, TurnToHeadingParams params, bool async) {
        if (params.forwards == false) {
            heading = normalizeAngle(heading + 180);
        }
        if (async) {
            if (asyncTask != nullptr) {
                cancelMotion();
            }
            asyncTask = new pros::Task([heading, timeout, params]() {
                turnToHeadingSync(heading, timeout, params);
            });
        } else {
            turnToHeadingSync(heading, timeout, params);
        }
    }

    void swingToHeadingSync(double heading, int timeout, bool lockLeft, TurnToHeadingParams params) {
        motionRunning = true;
        motionCancelled = false;

        double prevError = normalizeAngle(heading - odom::getHeading());
        double integral = 0;
        double prevOutput = 0;

        const int loopMs = 10;
        int startTime = pros::millis();

        while (pros::millis() - startTime < timeout && !motionCancelled) {
            pros::delay(loopMs);

            double error = normalizeAngle(heading - odom::getHeading());

            // Check early exit
            if (params.earlyExitRange > 0 && fabs(error) < params.earlyExitRange) {
                break;
            }

            // PID
            integral += error * loopMs;
            double derivative = (error - prevError) / (double)loopMs;
            double output = angularConsts.kP * error + angularConsts.kI * integral + angularConsts.kD * derivative;

            // Clamp
            output = clamp(output, -params.maxSpeed, params.maxSpeed);

            // Apply minimum speed on the control output
            if (fabs(output) < params.minSpeed && fabs(output) > 0.1) {
                output = (output > 0) ? params.minSpeed : -params.minSpeed;
            }

            // Rate limit
            double maxDelta = angularConsts.maxAccel * (loopMs / 1000.0);
            double delta = clamp(output - prevOutput, -maxDelta, maxDelta);
            output = prevOutput + delta;

            // Compute wheel speeds with one side locked. Preserve original sign conventions.
            double leftSpeed = 0.0;
            double rightSpeed = 0.0;
            if (lockLeft) {
                leftSpeed = 0.0;
                rightSpeed = -2.0 * output;
            } else {
                leftSpeed = 2.0 * output;
                rightSpeed = 0.0;
            }

            // Clamp wheel speeds to allowed range
            leftSpeed = clamp(leftSpeed, -params.maxSpeed, params.maxSpeed);
            rightSpeed = clamp(rightSpeed, -params.maxSpeed, params.maxSpeed);

            chass::drive127(leftSpeed, rightSpeed);

            prevError = error;
            prevOutput = output;
        }

        // ramp down gracefully for the moving side
        {
            const int loopMs2 = 10;
            double maxDelta = angularConsts.maxAccel * (loopMs2 / 1000.0);
            double prevOut = prevOutput;
            while (fabs(prevOut) > 1.0) {
                double delta = (0.0 - prevOut);
                if (delta > maxDelta)
                    delta = maxDelta;
                if (delta < -maxDelta)
                    delta = -maxDelta;
                prevOut += delta;
                double l = 0.0, r = 0.0;
                if (lockLeft) {
                    l = 0.0;
                    r = -2.0 * prevOut;
                } else {
                    l = 2.0 * prevOut;
                    r = 0.0;
                }
                l = clamp(l, -127, 127);
                r = clamp(r, -127, 127);
                chass::drive127(l, r);
                pros::delay(loopMs2);
            }
            chass::drive127(0, 0);
        }

        motionRunning = false;
    }

    void swingToHeading(double heading, int timeout, bool lockLeft, TurnToHeadingParams params, bool async) {
        if (async) {
            if (asyncTask != nullptr) {
                cancelMotion();
            }
            asyncTask = new pros::Task([heading, timeout, params, lockLeft]() {
                swingToHeadingSync(heading, timeout, lockLeft, params);
            });
        } else {
            swingToHeadingSync(heading, timeout, lockLeft, params);
        }
    }

    void turnToPoint(double x, double y, int timeout, TurnToHeadingParams params, bool async) {
        double targetHeading = odom::angleTo(x, y);
        turnToHeading(targetHeading, timeout, params, async);
    }

    void swingToPoint(double x, double y, int timeout, bool lockLeft, TurnToHeadingParams params, bool async) {
        double targetHeading = odom::angleTo(x, y);
        swingToHeading(targetHeading, timeout, lockLeft, params, async);
    }

    void moveToPoseSync(double x, double y, double theta, int timeout, MoveToPoseParams params) {
        motionRunning = true;
        motionCancelled = false;

        const int loopMs = 10;
        int startTime = pros::millis();

        double prevLinearOutput = 0;
        double prevAngularOutput = 0;

        while (pros::millis() - startTime < timeout && !motionCancelled) {
            pros::delay(loopMs);

            double distance = odom::distanceTo(x, y);

            // Check early exit
            if (params.earlyExitRange > 0 && distance < params.earlyExitRange) {
                break;
            }

            // Boomerang controller: create a carrot point
            double targetAngleRad = theta * std::numbers::pi / 180.0;
            double carrotX = x - params.lead * distance * std::sin(targetAngleRad);
            double carrotY = y - params.lead * distance * std::cos(targetAngleRad);

            // Calculate angle to carrot
            double carrotAngle = odom::angleTo(carrotX, carrotY);
            if (!params.forwards) {
                carrotAngle = normalizeAngle(carrotAngle + 180);
            }

            double angularError = normalizeAngle(carrotAngle - odom::getHeading());

            // Linear output based on distance
            double linearOutput = linearConsts.kP * distance;
            linearOutput = clamp(linearOutput, -params.maxSpeed, params.maxSpeed);

            // Angular output
            double angularOutput = angularConsts.kP * angularError;
            angularOutput = clamp(angularOutput, -params.maxSpeed, params.maxSpeed);

            // Apply minimum speed
            if (fabs(linearOutput) < params.minSpeed && fabs(linearOutput) > 0.1) {
                linearOutput = (linearOutput > 0) ? params.minSpeed : -params.minSpeed;
            }

            // Rate limit
            double maxLinearDelta = linearConsts.maxAccel * (loopMs / 1000.0);
            linearOutput = prevLinearOutput + clamp(linearOutput - prevLinearOutput, -maxLinearDelta, maxLinearDelta);

            double maxAngularDelta = angularConsts.maxAccel * (loopMs / 1000.0);
            angularOutput = prevAngularOutput + clamp(angularOutput - prevAngularOutput, -maxAngularDelta, maxAngularDelta);

            // Calculate wheel speeds
            double leftSpeed = linearOutput + angularOutput;
            double rightSpeed = linearOutput - angularOutput;

            // Normalize if exceeding max
            double maxOutput = std::max(fabs(leftSpeed), fabs(rightSpeed));
            if (maxOutput > params.maxSpeed) {
                leftSpeed = leftSpeed / maxOutput * params.maxSpeed;
                rightSpeed = rightSpeed / maxOutput * params.maxSpeed;
            }

            // Apply direction
            if (!params.forwards) {
                leftSpeed = -leftSpeed;
                rightSpeed = -rightSpeed;
            }

            chass::drive127(leftSpeed, rightSpeed);

            prevLinearOutput = linearOutput;
            prevAngularOutput = angularOutput;
        }

        // Final turn to target heading if needed
        if (!motionCancelled) {
            turnToHeadingSync(theta, 500, {.maxSpeed = params.maxSpeed, .minSpeed = params.minSpeed, .earlyExitRange = 2});
        }

        chass::drive127(0, 0);
        motionRunning = false;
    }

    void moveToPose(double x, double y, double theta, int timeout, MoveToPoseParams params, bool async) {
        if (async) {
            if (asyncTask != nullptr) {
                cancelMotion();
            }
            asyncTask = new pros::Task([x, y, theta, timeout, params]() {
                moveToPoseSync(x, y, theta, timeout, params);
            });
        } else {
            moveToPoseSync(x, y, theta, timeout, params);
        }
    }

    void followPathSync(const std::vector<std::pair<double, double>> &path, double lookahead, int timeout, double maxSpeed) {
        if (path.empty())
            return;

        motionRunning = true;
        motionCancelled = false;

        const int loopMs = 10;
        int startTime = pros::millis();
        size_t currentWaypoint = 0;

        double prevLinearOutput = 0;
        double prevAngularOutput = 0;

        while (pros::millis() - startTime < timeout && !motionCancelled && currentWaypoint < path.size()) {
            pros::delay(loopMs);

            double currentX = odom::getX();
            double currentY = odom::getY();

            // Find lookahead point
            double lookaheadX = path[currentWaypoint].first;
            double lookaheadY = path[currentWaypoint].second;

            // Progress through waypoints
            while (currentWaypoint < path.size() - 1) {
                double dx = path[currentWaypoint].first - currentX;
                double dy = path[currentWaypoint].second - currentY;
                double dist = std::sqrt(dx * dx + dy * dy);

                if (dist < lookahead) {
                    currentWaypoint++;
                } else {
                    break;
                }
            }

            // Interpolate lookahead point along path segment
            if (currentWaypoint < path.size()) {
                lookaheadX = path[currentWaypoint].first;
                lookaheadY = path[currentWaypoint].second;
            }

            // Calculate curvature to lookahead point
            double dx = lookaheadX - currentX;
            double dy = lookaheadY - currentY;
            double distance = std::sqrt(dx * dx + dy * dy);

            // Check if we've reached the end
            if (currentWaypoint >= path.size() - 1 && distance < lookahead / 2) {
                break;
            }

            // Calculate angle to lookahead
            double targetAngle = odom::angleTo(lookaheadX, lookaheadY);
            double angularError = normalizeAngle(targetAngle - odom::getHeading());

            // Pure pursuit curvature: 2 * sin(error) / distance
            double curvature = (distance > 0.01) ? (2.0 * std::sin(angularError * std::numbers::pi / 180.0) / distance) : 0;

            // Calculate wheel speeds
            double linearOutput = maxSpeed * (1.0 - std::abs(curvature) * 5);
            linearOutput = clamp(linearOutput, maxSpeed * 0.3, maxSpeed);

            double angularOutput = curvature * linearOutput * 10;
            angularOutput = clamp(angularOutput, -maxSpeed, maxSpeed);

            // Rate limit
            double maxLinearDelta = linearConsts.maxAccel * (loopMs / 1000.0);
            linearOutput = prevLinearOutput + clamp(linearOutput - prevLinearOutput, -maxLinearDelta, maxLinearDelta);

            double maxAngularDelta = angularConsts.maxAccel * (loopMs / 1000.0);
            angularOutput = prevAngularOutput + clamp(angularOutput - prevAngularOutput, -maxAngularDelta, maxAngularDelta);

            double leftSpeed = linearOutput + angularOutput;
            double rightSpeed = linearOutput - angularOutput;

            // Normalize
            double maxOutput = std::max(fabs(leftSpeed), fabs(rightSpeed));
            if (maxOutput > maxSpeed) {
                leftSpeed = leftSpeed / maxOutput * maxSpeed;
                rightSpeed = rightSpeed / maxOutput * maxSpeed;
            }

            chass::drive127(leftSpeed, rightSpeed);

            prevLinearOutput = linearOutput;
            prevAngularOutput = angularOutput;
        }

        chass::drive127(0, 0);
        motionRunning = false;
    }

    void followPath(const std::vector<std::pair<double, double>> &path, double lookahead, int timeout, double maxSpeed, bool async) {
        if (async) {
            if (asyncTask != nullptr) {
                cancelMotion();
            }
            // Need to copy the path for async
            std::vector<std::pair<double, double>> pathCopy = path;
            asyncTask = new pros::Task([pathCopy, lookahead, timeout, maxSpeed]() {
                followPathSync(pathCopy, lookahead, timeout, maxSpeed);
            });
        } else {
            followPathSync(path, lookahead, timeout, maxSpeed);
        }
    }

    void cancelMotion() {
        motionCancelled = true;
        if (asyncTask != nullptr) {
            // Wait for task to finish
            while (motionRunning) {
                pros::delay(10);
            }
            delete asyncTask;
            asyncTask = nullptr;
        }
        chass::drive127(0, 0);
    }

    bool isMoving() {
        return motionRunning;
    }
} // namespace pid