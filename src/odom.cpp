#include "odom.hpp"
#include "main.h"
#include "robot.hpp"
#include "sensors.hpp"
#include <atomic>
#include <cmath>
#include <numbers>
namespace odom {
    // constants
    const double loopDelay = 10;             // loop delay in msecs
    const double trackWidth = 12.15625;      // track width in inches
    const double wheelRadius = 1.625;        // 3.25 inch wheels
    const double encoderWheelRadius = 1.375; // 2.75 inch tracking wheels (radius = 1.375 inch)
    const double scalingF = 24.0 / 11.2;     // scaling factor for odom (empirically determined)

    // main tracking variables
    double xpos = 0,
           ypos = 0,
           prevtheta = 0,
           prevHorizPos = 0,
           prevVertPos = 0;
    // auxilliary variables
    double xvelo = 0,
           yvelo = 0,
           speed = 0,
           angvelo = 0;
    double newxpos = 0,
           newypos = 0,
           newtheta = 0;
    static std::atomic<bool> settingPose{false};

    double getVertPos() {
        return bot::vertEnc.get_position() / 36000.0 * std::numbers::pi * encoderWheelRadius * scalingF;
    }

    double getHorizPos() {
        return bot::horizEnc.get_position() / 36000.0 * std::numbers::pi * encoderWheelRadius * scalingF;
    }

    void odomLoop() {
        pros::delay(loopDelay);
        if (settingPose.load()) {
            xpos = newxpos;
            ypos = newypos;
            bot::imu.set_rotation(newtheta);
            // Give the IMU a moment to apply the heading, then read the actual reported heading
            pros::delay(5);
            double actualHeading = bot::getRotation();
            // Use the IMU's reported heading for prevtheta so odom doesn't see a large rotation jump
            prevtheta = -actualHeading / 180.0 * std::numbers::pi;
            // Also update newtheta to reflect what the IMU actually reports
            newtheta = actualHeading;
            // Reset tracking wheel positions to current readings (include scalingF like main loop)
            prevHorizPos = getHorizPos();
            prevVertPos = getVertPos();
            settingPose.store(false);
            return;
        }

        // Get IMU heading
        double theta = bot::getRotation() / 180.0 * std::numbers::pi; // convert to radians

        theta = -theta; // invert to match coordinate system

        double deltatheta = theta - prevtheta;
        prevtheta = theta;

        angvelo = deltatheta * 1000.0 / loopDelay;

        // Get tracking wheel positions in centidegrees, convert to inches
        // inches = centidegrees * π radians / (360 * 100) * radius = centidegrees * π / 36000 * radius
        double horizPos = getHorizPos();
        double vertPos = getVertPos();

        // Calculate raw deltas
        double deltaHoriz = horizPos - prevHorizPos;
        double deltaVert = vertPos - prevVertPos;

        prevHorizPos = horizPos;
        prevVertPos = vertPos;

        // Account for rotation's effect on tracking wheels (arc motion correction)
        // When robot rotates, the tracking wheels move in an arc around the center of rotation
        // deltaLocal = deltaEncoder - offset * deltatheta
        double deltaXlocal = deltaHoriz - bot::horizOffset * deltatheta;
        double deltaYlocal = deltaVert - bot::vertOffset * deltatheta;

        // Transform to global coordinates using average heading during the interval
        double avgTheta = theta - deltatheta / 2.0;
        double c = std::cos(avgTheta), s = std::sin(avgTheta);

        // Rotation matrix transformation
        double deltaX = deltaXlocal * c - deltaYlocal * s;
        double deltaY = deltaXlocal * s + deltaYlocal * c;

        xvelo = deltaX * 1000.0 / loopDelay;
        yvelo = deltaY * 1000.0 / loopDelay;
        speed = sqrt(xvelo * xvelo + yvelo * yvelo);

        xpos += deltaX;
        ypos += deltaY;

        // std::cout << "deltaHoriz: " << deltaHoriz << ", deltaVert: " << deltaVert << ", deltatheta: " << deltatheta << ", deltaXlocal: " << deltaXlocal << ", deltaYlocal: " << deltaYlocal << ", deltaX: " << deltaX << ", deltaY: " << deltaY << ", xpos: " << xpos << ", ypos: " << ypos << std::endl;
    }
    pros::Task *odomTask;
    void initialize() {
        // prime previous readings before starting the task
        // match odomLoop sign convention (theta is later negated) and include scalingF for encoder positions
        prevtheta = -bot::getRotation() / 180.0 * std::numbers::pi;
        prevHorizPos = bot::horizEnc.get_position() / 36000.0 * std::numbers::pi * encoderWheelRadius * scalingF;
        prevVertPos = bot::vertEnc.get_position() / 36000.0 * std::numbers::pi * encoderWheelRadius * scalingF;

        odomTask = new pros::Task([]() {while(1) odomLoop(); });
        xvelo = 0;
        yvelo = 0;
        speed = 0;
        angvelo = 0;
        xpos = 0;
        ypos = 0;
    }

    double getX() {
        return xpos;
    }

    double getY() {
        return ypos;
    }

    double getHeading() {
        return bot::getRotation();
    }

    void setPose(double x, double y, double heading) {
        settingPose.store(true);
        newxpos = x;
        newypos = y;
        newtheta = heading;
        pros::delay(loopDelay); // wait for odom loop to set the pose
    }

    void resetPose() {
        setPose(0, 0, 0);
    }

    double distanceTo(double targetX, double targetY) {
        double dx = targetX - xpos;
        double dy = targetY - ypos;
        return std::sqrt(dx * dx + dy * dy);
    }

    double angleTo(double targetX, double targetY) {
        double dx = targetX - xpos;
        double dy = targetY - ypos;
        // atan2 gives angle in radians, convert to degrees
        // Adjust so 0 degrees is forward (positive Y direction)
        double angleRad = std::atan2(dx, dy);
        return angleRad * 180.0 / std::numbers::pi;
    }

    void debugPrint() {
        std::cout << "X, Y: ( " << getX() << ", " << getY() << " ), Heading: " << getHeading() << std::endl;
    }
} // namespace odom