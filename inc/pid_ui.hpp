#pragma once

#include <chrono>
#include <string>
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

    PidUi(const PidSettings& initial, float setpointLimitCm);
    PidUi(const PidUi&) = delete;
    PidUi& operator=(const PidUi&) = delete;

    const PidSettings& active() const { return active_; }
    bool running() const { return running_; }
    bool takeRestart();

    void setCanRun(bool canRun);
    void log(Clock::time_point t, float x, float y, float tiltX, float tiltY, bool controlling);

    void draw();
    bool handleKey(int key);

    cv::Mat renderPanel() const;
    cv::Mat renderPlot() const;

private:
    struct Row {
        std::string label;
        float PidSettings::*field;
        float sliderMin, sliderMax;
        float hardMin, hardMax;
        const char* fmt;
    };
    struct Sample {
        double t;
        float x, y, spX, spY;
        float tiltX, tiltY;
        bool controlling;
    };

    static void mouseThunk(int event, int x, int y, int flags, void* self);
    void onMouse(int event, int x, int y, int flags);
    void setFromSlider(int row, int x);
    void commitEdit();
    void apply();
    void resetIntegral();
    void toggleRunning();
    void togglePlot();
    bool hasPending() const;
    double nowSec() const;

    std::vector<Row> rows_;
    PidSettings active_, pending_;
    bool running_ = false, canRun_ = false, restart_ = false;
    bool plotOpen_ = false;
    int plotFrames_ = 0;
    int dragging_ = -1, editRow_ = -1;
    std::string editText_;
    std::vector<Sample> log_;
    Clock::time_point start_;
};