#include "odom.hpp"
#include "main.h"
#include "robot.hpp"
#include "sensors.hpp"
#include <cmath>
#include <numbers>
namespace odom {
    // constants
    const double loopDelay = 10;        // loop delay in msecs
    const double trackWidth = 12.15625; // track width in inches
    const double wheelRadius = 1.625;   // 3.25 inch wheels
    const double encoderWheelRadius = 1.0; // 2 inch tracking wheels (radius = 1 inch)

    // main tracking variables
    double xpos = 0,
           ypos = 0,
           prevtheta = 0,
           prevLpos = 0,
           prevRpos = 0,
           prevHorizPos = 0,
           prevVertPos = 0;
    // auxilliary variables
    double xvelo = 0,
           yvelo = 0,
           speed = 0,
           angvelo = 0;
    void odomLoop() {
        pros::delay(loopDelay);

        // Get IMU heading
        double theta = bot::imu.get_rotation() / 180.0 * std::numbers::pi; // convert to radians
        double deltatheta = theta - prevtheta;
        prevtheta = theta;

        angvelo = deltatheta * 1000.0 / loopDelay;

        // Get tracking wheel positions in centidegrees, convert to inches
        // inches = centidegrees * π radians / 18000 * radius
        double horizPos = bot::horizEnc.get_position() / 18000.0 * std::numbers::pi * encoderWheelRadius;
        double vertPos = bot::vertEnc.get_position() / 18000.0 * std::numbers::pi * encoderWheelRadius;
        
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
        prevtheta = bot::imu.get_rotation() / 180.0 * std::numbers::pi;
        prevHorizPos = bot::horizEnc.get_position() / 18000.0 * std::numbers::pi * encoderWheelRadius;
        prevVertPos = bot::vertEnc.get_position() / 18000.0 * std::numbers::pi * encoderWheelRadius;

        odomTask = new pros::Task([]() {while(1) odomLoop(); });
        xvelo = 0,
        yvelo = 0,
        speed = 0,
        angvelo = 0;
        xpos = 0,
        ypos = 0;
    }

    double getX() {
        return xpos;
    }

    double getY() {
        return ypos;
    }

    double getHeading() {
        return bot::imu.get_rotation();
    }

    void setPose(double x, double y, double heading) {
        xpos = x;
        ypos = y;
        bot::imu.set_heading(heading);
        prevtheta = heading / 180.0 * std::numbers::pi;
        // Reset tracking wheel positions to current readings
        prevHorizPos = bot::horizEnc.get_position() / 18000.0 * std::numbers::pi * encoderWheelRadius;
        prevVertPos = bot::vertEnc.get_position() / 18000.0 * std::numbers::pi * encoderWheelRadius;
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
} // namespace odom