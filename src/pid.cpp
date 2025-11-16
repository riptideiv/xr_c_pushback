#include "pid.hpp"
#include "chassis.hpp"
#include "sensors.hpp"

namespace pid {
    PIDConsts linearConsts(6.25, 0.0, 75, 1500);
    PIDConsts angularConsts(1.875, 0.0, 143.75, 10000000);

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
} // namespace pid