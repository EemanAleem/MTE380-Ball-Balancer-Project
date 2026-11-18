#pragma once
#include <cstdint>
#include <stdbool.h>
#include <algorithm>

class PID {
    public:
        PID(double kp, double ki, double kd, double tau);
        double calculateControlEffort(double setpoint, double measured, double dt);

    private:
        double kp, ki, kd;
        double integral, derivative;
        double tau;
        double previous_measured;
        bool first_run;
};