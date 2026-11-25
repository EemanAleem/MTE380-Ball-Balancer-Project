#pragma once
#include <chrono>
#include <vector>
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
    void setTrajectory(const std::vector<cv::Point2f>& waypointsCm, int current);

private:
    cv::VideoCapture cap_;
    cv::Size size_;
    std::vector<cv::Point2f> traj_;
    int trajIdx_ = -1;
};