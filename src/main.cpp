#include <array>
#include <chrono>
#include <cstdio>
#include <opencv2/highgui.hpp>

#include "ball_tracker.hpp"
#include "ik.hpp"
#include "pid.hpp"
#include "pid_ui.hpp"
#include "servo_link.hpp"


// Constants
static const int CAMERA_INDEX = 0;

static const char* SERIAL_PORT = "COM3";
static const int SERIAL_BAUD = 9600;
static const bool ECHO_ARDUINO = true;

static const std::array<int, NUM_SERVOS> SERVO_CHANNELS = {3, 5, 7};

static const PidSettings INITIAL_PID = {0.4f, 0.1f, 0.25f, 0.03f, 0.0f, 0.0f};
static const float SETPOINT_LIMIT_CM = 30.0f;
static const Vec2 SETPOINT_DEFAULT = {0.0, 0.0}; // x and y setpoints in cm

int main() {
    using Clock = std::chrono::steady_clock;

    ServoLink servos;
    if (!servos.open(SERIAL_PORT, SERIAL_BAUD, SERVO_CHANNELS))
    {
        // std::printf("ERROR: could not connect to motors");
        // return 1;
    }

    BallTracker tracker;
    if (!tracker.open(CAMERA_INDEX)) {
        // std::printf("ERROR: could not connect to camera");
        // return 1;
    }

    PidUi ui(INITIAL_PID, SETPOINT_LIMIT_CM);

    // Define controllers for x and y directions
    PID pidX(INITIAL_PID.kp, INITIAL_PID.ki, INITIAL_PID.kd, INITIAL_PID.tau);
    PID pidY(INITIAL_PID.kp, INITIAL_PID.ki, INITIAL_PID.kd, INITIAL_PID.tau);

    // Flags
    bool controlling = false; // Is the PID currently running? True -> Use the current values, False -> Start fresh
    bool levelSent = false; // Is the platform level? True -> Do not send unnecessary level commands to Arduino, False -> Send level command to Arduino
    Clock::time_point pastBallTime; // Time of the last frame

    // Levelling the platform
    auto holdLevel = [&]() {
        if (!levelSent) {
            servos.send(inverseKinematics(0.0f, 0.0f));
            levelSent = true;
        }
    };

    holdLevel();
    BallReading ball;

    while (true) {
        // 0. Update past frame time
        pastBallTime = ball.time;
        
        // 1. Declare Ball Position struct
        bool visible = tracker.update(ball);
        double dt = ball.time - pastBallTime;

        // 2. Update PID UI Tuning
        ui.setCanRun(tracker.calibrated());
        const PidSettings& s = ui.active();

        bool ranPid = false;
        if (visible) {
            // 3. Apply PID Controller
            Vec2 uVec; 
            uVec[0] = pidX.calculateControlEffort(SETPOINT_DEFAULT[0], BallReading.x_cm, dt);
            uVec[1] = pidY.calculateControlEffort(SETPOINT_DEFAULT[1], BallReading.y_cm, dt);

            // 4. Calculate (Actuating) Absolute Motor Angles
            Vec3 motorAngles = calculateMotorAngles(uVec);
            levelSent = false;
            ranPid = true;

            // 5. Show on UI
            ui.log(ball.time, ball.x_cm, ball.y_cm, tiltX, tiltY, ranPid);
            ui.draw();
        }
        else {
            holdLevel();
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