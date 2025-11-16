#pragma once

namespace utils {
    int sign(double val);

    // PID Tuning Functions
    void runFwdBwdTest(double target, int timeout);
    void runTurnTest(double target, int timeout);
    void runAngularPID_kPs(double kD, double begin, double end, double step, double target, int timeout);
    void runAngularPID_kDs(double kP, double begin, double end, double step, double target, int timeout);
    void runLateralPID_kPs(double kD, double begin, double end, double step, double target, int timeout);
    void runLateralPID_kDs(double kP, double begin, double end, double step, double target, int timeout);
    void runLateralBSearchkP(double kD, double l, double r, double target, int timeout);
    void runLateralBSearchkD(double kP, double l, double r, double target, int timeout);
    void runAngularBSearchkP(double kD, double l, double r, double target, int timeout);
    void runAngularBSearchkD(double kP, double l, double r, double target, int timeout);
} // namespace utils
