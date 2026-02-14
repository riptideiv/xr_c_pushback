#include "sensors.hpp"

namespace bot {
    pros::Imu imu1(3);
    pros::Imu imu2(9);
    pros::Rotation horizEnc(19), vertEnc(15);
    const double horizOffset = -2.11287, vertOffset = -1.85595; // horiz offset = rightward is +, vert offset = forward is +

    // Correction factor to map IMU-reported rotation to actual rotation.
    // Example: sensor reads 358 when actual is 360 -> correction = 360.0/358.0
    static const double imuRotationCorrection1 = 360.0 / 358.0;
    static const double imuRotationCorrection2 = 360.0 / 358.0;

    struct KalmanFilter {
        double x = 0;    // State estimate
        double P = 0;    // Error covariance
        double Q = 0.01; // Process noise covariance
        double R = 0.1;  // Measurement noise covariance

        void predict() {
            P = P + Q;
        }

        void update(double measurement) {
            double K = P / (P + R);
            x = x + K * (measurement - x);
            P = (1 - K) * P;
        }

        void setState(double state) {
            x = state;
            P = 0;
        }
    } kf;

    void initialize_sensors() {
        horizEnc.reset();
        vertEnc.reset();
        imu1.reset(true);
        imu2.reset(true);
    }

    void setRotation(double rotation) {
        imu1.set_rotation(rotation);
        imu2.set_rotation(rotation);
        kf.setState(rotation);
    }

    void setHeading(double heading) {
        imu1.set_heading(heading);
        imu2.set_heading(heading);
        // Also update correlation to rotation if possible, but heading wraps 360.
        // We set KF state to match.
        kf.setState(heading);
    }

    // Return corrected IMU rotation in degrees
    double getRotation() {
        double z1 = imu1.get_rotation() * imuRotationCorrection1;
        double z2 = imu2.get_rotation() * imuRotationCorrection2;

        kf.predict();
        kf.update(z1);
        kf.update(z2);

        return kf.x;
    }
} // namespace bot