#include "pid.hpp"

// Global variables for PID controller
constexpr float LOWER_INTEGRAL_LIMIT = -0.5f;
constexpr float UPPER_INTEGRAL_LIMIT = 0.5f;
constexpr float LOWER_OUTPUT_LIMIT = -10.0f;
constexpr float UPPER_OUTPUT_LIMIT = 10.0f;

// Constructor
PID::PID(float kp, float ki, float kd, float tau)
    : kp(kp), ki(ki), kd(kd), tau(tau),
      integral(0.0f), derivative(0.0f),
      previous_measured(0.0f), first_run(true) {}

float PID::update(float setpoint, float measured, float dt) {
    // 0. Ensure dt is positive to avoid division by zero
    if (dt <= 0.0f) dt = 1e-3f;

    // 1. Error Delta Calculation
    float error = setpoint - measured;
    if (first_run) {
        previous_measured = measured;
        first_run = false;
    }

    // 2. Proportional Control
    float proportional = kp * error;

    // 3. Integral Control + Anti-Windup Mechanism
    integral += ki * error * dt;
    integral = std::clamp(integral, LOWER_INTEGRAL_LIMIT, UPPER_INTEGRAL_LIMIT);

    // 4. Derivative Control + Derivative Kick
    float numerator = -2.0f * kd * (measured - previous_measured) + (2.0f * tau - dt) * derivative;
    float denominator = 2.0f * tau + dt;
    derivative = numerator / denominator;

    // 5. Final Control Effort Calculation + Update Previous Error
    float control_effort = proportional + integral + derivative;
    control_effort = std::clamp(control_effort, LOWER_OUTPUT_LIMIT, UPPER_OUTPUT_LIMIT);
    
    // 6. Update persistent variables
    previous_measured = measured;

    // 7. Output
    return control_effort;
}