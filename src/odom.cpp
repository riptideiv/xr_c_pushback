#include "odom.hpp"
#include "main.h"
#include "robot.hpp"
#include <cmath>
namespace odom {
    // constants
    const double loopDelay = 10;        // loop delay in msecs
    const double trackWidth = 12.15625; // track width in inches
    const double wheelRadius = 1.625;   // 3.25 inch wheels

    // main tracking variables
    double xpos = 0,
           ypos = 0,
           prevtheta = 0,
           prevLpos = 0,
           prevRpos = 0;
    // auxilliary variables
    double xvelo = 0,
           yvelo = 0,
           speed = 0,
           angvelo = 0;
    void odomLoop() {
        pros::delay(loopDelay);

        // inches = degrees * π radians / 180 degrees * inches
        double posL = chass::getLeftPos() / 180.0 * std::numbers::pi * wheelRadius,
               posR = chass::getRightPos() / 180.0 * std::numbers::pi * wheelRadius,
               theta = bot::imu.get_rotation() / 180.0 * std::numbers::pi; // convert to radians
        double deltaL = posL - prevLpos, deltaR = posR - prevRpos;
        double deltatheta = theta - prevtheta;
        prevLpos = posL;
        prevRpos = posR;
        prevtheta = theta;

        angvelo = deltatheta * 1000.0 / loopDelay;

        double deltaXlocal, deltaYlocal;
        if (fabs(deltatheta) < 0.001) { // didn't turn
            deltaXlocal = 0;
            deltaYlocal = (deltaL + deltaR) / 2.0;
        } else {
            double arcradius = (deltaR + deltaL) / (2.0 * deltatheta);

            deltaXlocal = arcradius * (1.0 - std::cos(deltatheta));
            deltaYlocal = arcradius * std::sin(deltatheta);
        }

        double avgTheta = theta - deltatheta / 2.0;
        double c = std::cos(avgTheta), s = std::sin(avgTheta);

        // multiply by rotation matrix of -theta
        double deltaX = deltaXlocal * c - deltaYlocal * s;
        double deltaY = deltaXlocal * s + deltaYlocal * c;

        xvelo = deltaX * 1000.0 / loopDelay;
        yvelo = deltaY * 1000.0 / loopDelay;
        speed = sqrt(xvelo * xvelo + yvelo * yvelo);

        xpos += deltaX;
        ypos += deltaY;

        // std::cout << "deltaL: " << deltaL << ", deltaR: " << deltaR << ", deltatheta: " << deltatheta << ", deltaXlocal: " << deltaXlocal << ", deltaYlocal: " << deltaYlocal << ", deltaX: " << deltaX << ", deltaY: " << deltaY << ", xpos: " << xpos << ", ypos: " << ypos << std::endl;
    }
    pros::Task *odomTask;
    void initialize() {
        // prime previous readings before starting the task
        prevLpos = chass::getLeftPos() / 180.0 * std::numbers::pi * wheelRadius;
        prevRpos = chass::getRightPos() / 180.0 * std::numbers::pi * wheelRadius;
        prevtheta = bot::imu.get_rotation() / 180.0 * std::numbers::pi;

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