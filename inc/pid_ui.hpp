#pragma once

#include <chrono>
#include <string>
#include <functional>
#include <vector>
#include <opencv2/core.hpp>

struct PidSettings {
    float kp, ki, kd;
    float tau;
    float setX, setY;
};

class PidUi {
public:
    using Clock = std::chrono::steady_clock;

    PidUi(float kp, float ki, float kd, float setX = 0.0f, float setY = 0.0f, float setpointLimitCm = 10.0f);
    PidUi(const PidUi&) = delete;
    PidUi& operator=(const PidUi&) = delete;

    std::function<void(float kp, float ki, float kd)> onApply;

    float setpointX() const { return active_.setX; }
    float setpointY() const { return active_.setY; }

    void log(Clock::time_point t, float x, float y, float tiltX, float tiltY);
    void draw();
    bool handleKey(int key);

private:
    struct Values { float kp, ki, kd, setX, setY; };
    struct Row { const char* label; float Values::*field; float sliderMin, sliderMax, hardMin, hardMax; };
    struct Sample { double t; float x, y, spX, spY, tiltX, tiltY; };

    static void mouseThunk(int event, int x, int y, int flags, void* self);
    void onMouse(int event, int x, int y, int flags);
    void setFromSlider(int row, int x);
    void commitEdit();
    void apply();
    cv::Mat renderPanel() const;
    cv::Mat renderPlot() const;

    Values pending_, active_;
    std::vector<Row> rows_;
    int dragging_ = -1, editRow_ = -1;
    std::string editText_;
    bool plotRequested_ = false;
    std::vector<Sample> log_;
    Clock::time_point start_;

    // Trajectory planning
    bool trajActive_ = false;
    int trajIdx_ = 0;
    Clock::time_point trajNext_;
    
    void startTrajectory();
    void stopTrajectory();
    void goToWaypoint(int idx);
    void updateTrajectory();
};