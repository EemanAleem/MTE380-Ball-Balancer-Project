#include "pid.hpp"

// Global variables for PID controller
constexpr double LOWER_INTEGRAL_LIMIT = -0.5;
constexpr double UPPER_INTEGRAL_LIMIT = 0.5;
constexpr double LOWER_OUTPUT_LIMIT = -10.0;
constexpr double UPPER_OUTPUT_LIMIT = 10.0;

// Constructor
PID::PID(double kp, double ki, double kd, double tau)
    : kp(kp), ki(ki), kd(kd), tau(tau),
      integral(0.0), derivative(0.0),
      previous_measured(0.0), first_run(true) {}

double PID::calculateControlEffort(double setpoint, double measured, double dt) {
    // 0. Ensure dt is positive to avoid division by zero
    if (dt <= 0.0) dt = 1e-3;

    // 1. Error Delta Calculation
    double error = setpoint - measured;
    if (first_run) {
        previous_measured = measured;
        first_run = false;
    }

    // 2. Proportional Control
    double proportional = kp * error;

    // 3. Integral Control + Anti-Windup Mechanism
    integral += ki * error * dt;
    integral = std::clamp(integral, LOWER_INTEGRAL_LIMIT, UPPER_INTEGRAL_LIMIT);

    // 4. Derivative Control + Derivative Kick
    double numerator = -2.0 * kd * (measured - previous_measured) + (2.0 * tau - dt) * derivative;
    double denominator = 2.0 * tau + dt;
    derivative = numerator / denominator;

    // 5. Final Control Effort Calculation + Update Previous Error
    double control_effort = proportional + integral + derivative;
    control_effort = std::clamp(control_effort, LOWER_OUTPUT_LIMIT, UPPER_OUTPUT_LIMIT);
    
    // 6. Update persistent variables
    previous_measured = measured;

    // 7. Output
    return control_effort;
}