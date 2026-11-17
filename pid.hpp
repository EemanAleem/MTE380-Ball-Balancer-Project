#pragma once
#include <cstdint>
#include <stdbool.h>
#include <algorithm>

class PID {
    public:
        PID(float kp, float ki, float kd, float tau);
        float update(float setpoint, float measured, float dt);

    private:
        float kp, ki, kd;
        float integral, derivative;
        float tau;
        float previous_measured;
        bool first_run;
};