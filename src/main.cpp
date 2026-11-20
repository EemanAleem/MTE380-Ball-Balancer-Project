#include <array>
#include <chrono>
#include <cstdio>
#include <opencv2/highgui.hpp>

#include "ball_tracker.hpp"
#include "ik.hpp"
#include "pid.hpp"
#include "pid_ui.hpp"
#include "servo_link.hpp"

using Clock = std::chrono::steady_clock;

// System dependent serial port name and camera index
static const int CAMERA_INDEX = 1; // If your laptop has a webcam, that is usually 0. So your external camera is likely 1.
static const char* SERIAL_PORT = "COM8"; // Choose COM port for Arduino.

static const bool ECHO_ARDUINO = true; // If you want msgs from Arduino to be printed to console = true.

// Constants
static const int SERIAL_BAUD = 115200;
static const std::array<int, NUM_SERVOS> SERVO_CHANNELS = {3, 5, 7};
static const PidSettings INITIAL_PID = {0.2f, 0.0f, 0.0f, 0.03f, 0.0f, 0.0f};
static const float SETPOINT_LIMIT_CM = 30.0f;
static const Vec2 SETPOINT_DEFAULT = {0.0, 0.0}; // x and y setpoints in cm

// Convert IK output (Vec3) to the format ServoLink::send expects
static std::array<float, NUM_SERVOS> toServoCmd(const Vec3& a) {
    return {static_cast<float>(a[0]), static_cast<float>(a[1]), static_cast<float>(a[2])};
}

int main() {
    // 1. Check connections to Arduino with servos and camera.
    ServoLink servos;
    if (!servos.open(SERIAL_PORT, SERIAL_BAUD, SERVO_CHANNELS))
    {
        std::printf("ERROR: could not connect to motors");
        return 1;
    }
    BallTracker tracker;
    if (!tracker.open(CAMERA_INDEX)) {
        std::printf("ERROR: could not connect to camera");
        return 1;
    }

    // 2. Set up PID Tuner UI
    PidUi ui(INITIAL_PID, SETPOINT_LIMIT_CM);

    // 3. Define PID controllers for x and y axes
    PID pidX(INITIAL_PID.kp, INITIAL_PID.ki, INITIAL_PID.kd, INITIAL_PID.tau);
    PID pidY(INITIAL_PID.kp, INITIAL_PID.ki, INITIAL_PID.kd, INITIAL_PID.tau);

    // 4. Boolean flags + vars
    bool visible = false;   // Is the ball detected? Yes = true, No = false.
    bool levelSent = false; // Is the platform level? True -> Do not send unnecessary level commands to Arduino
    Clock::time_point pastBallTime; // Time of the last frame
    BallReading ball; // Declare the ball reading var with x, y coords and timestamp.

    // 5. Create func + actually levelling the platform on startup.
    auto holdLevel = [&]() {
        servos.send(toServoCmd(calculateMotorAngles(Vec3{0.0, 0.0, 0.0})));
        levelSent = true;
    };
    holdLevel();

    while (true) {
        // 0. Update past frame time
        pastBallTime = ball.time;

        // 1. Declare Ball Position struct
        visible = tracker.update(ball);
        double dt = std::chrono::duration<double>(ball.time - pastBallTime).count(); // seconds

        // 2. Update PID UI Tuning
        ui.setCanRun(tracker.calibrated());
        const PidSettings& s = ui.active();

        if (visible) {
            // 3. Apply PID Controller
            double tiltX = pidX.calculateControlEffort(SETPOINT_DEFAULT[0], ball.x_cm, dt);
            double tiltY = pidY.calculateControlEffort(SETPOINT_DEFAULT[1], ball.y_cm, dt);
            Vec3 uVec{tiltX, tiltY, 0.0};

            // 4. Calculate (Actuating) Absolute Motor Angles and send them
            Vec3 motorAngles = calculateMotorAngles(uVec);
            servos.send(toServoCmd(motorAngles));
            levelSent = false;

            // 5. Show on UI
            ui.log(ball.time, ball.x_cm, ball.y_cm, tiltX, tiltY);
            ui.draw();
        }
        else {
            if (levelSent == false) {
                holdLevel();
                levelSent = true;
            }
        }
        servos.poll(ECHO_ARDUINO);

        int key = cv::waitKey(1);
        if (key >= 0) {
            key &= 0xFF;
            if (!ui.handleKey(key)) {
                if (key == 'q') break;
                tracker.handleKey(key);
            }
        }
    }

    levelSent = false;
    holdLevel();
    tracker.close();
    cv::destroyAllWindows();
    return 0;
}