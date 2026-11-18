#pragma once
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

struct BallReading {
    float x_cm = 0.0f;
    float y_cm = 0.0f;
    std::chrono::steady_clock::time_point time;
};

class BallTracker {
public:
    bool open(int cameraIndex);

    bool update(BallReading& out);

    void handleKey(int key);

    bool calibrated() const;
    void close();

private:
    cv::VideoCapture cap_;
    cv::Size size_;
};
